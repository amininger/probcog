package magic.slam;

import java.awt.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;

import javax.swing.*;
import javax.imageio.*;

import april.vis.*;
import april.jcam.*;
import april.util.*;
import april.jmat.*;
import april.image.*;
import april.image.corner.*;
import april.config.*;
import april.lcmtypes.*;
import april.laser.scanmatcher.*;
import april.graph.*;
import april.jmat.ordering.*;

import lcm.lcm.*;
import magic.lcmtypes.*;
import magic.util.*;
import magic.obs.*;
import magic.vis.*;
import magic.gps.*;
import magic.lcm.*;
import magic.gslam.*;
import magic.laser.*;

public class TerrainMapper implements AX12PoseLidar.Listener, LCMSubscriber, GPSFixListener
{
    // XXX Globally available for debugging
    VisWorld vw = new VisWorld();

    Config config;
    GetOpt gopt;
    LCM lcm = LCM.getSingleton();

    double ground_slope = 0.18;

    double slam_zmin = 0.3; // ignore points just off the ground surface estimate
    double slam_zmax = 2.0; // ignore points this far above ground estimate (they're ceiling?)
    double slam_max_unsupported = 0.5; // ignore points that don't have stuff under them.

    double hazard_zmin = -2.0;
    double hazard_zmax = 1.5;
    double hazard_max_unsupported = 0.5;

    // which historical hazard maps should we use? 0 = current. Must
    // be from oldest to newest.
    int hazard_history[] = new int[] { -6, -4, -2, -1, 0 };
    double hazard_range = 15;

    int min_slices = 10;  // discard sweeps with fewer than this many slices.

    int max_sweeps = 100; // large numbers enable historical data for visualization, mostly.

    double obstacleHeight = 0.125;     // intracell min/max for HazardMap
    double maxJump = 0.5;             // intercell slope for HazardMap
    int niters = 2;                   // iterations
    AX12PoseLidar apl;

    robot_command_list_t cmd_list;
    long cmdsUtime; // needed to properly handle CMDS message
    boolean ensureMonotonicMsgs;  // only accept newer msge (no loops in log playback)

    ArrayList<Listener> listeners = new ArrayList<Listener>();
    ArrayList<Sweep> sweeps = new ArrayList<Sweep>();

    GarminNMEAParser gpsParser;
    ExpiringMessageCache<gps_t> gpsCache = new ExpiringMessageCache<gps_t>(1.0);

    static final int MIN_OBS_DELAY_US = 2000000;
    static final int MAX_GPS_AGE_US = 400000; // time it takes to get two GPS messages

    GridMapModel.Train train;
    int trainsizes[];

    Object datasQueueObject = new Object();
    ArrayList<AX12PoseLidar.Data> datasQueue = null;

    static final public class Sweep
    {
        long utime;  // utime of most recent lidar scan

        double xyt[]; // 3dof pose in local frame
        double xyt_slam[];

        ArrayList<Slice> slices = new ArrayList<Slice>();

        GroundModel ground;

        // selected points from each slice, aligned with respect to the first slice.
        ArrayList<double[]> slamPointsBody = new ArrayList<double[]>();

        GridMap slamgm; // binary map consisting of aligned slam points: body frame
        GridMap slamgm_blurred; // body frame
        GridMap aligngm; // gridmap built up during alignment process: body frame

        GridMap hazardgm; // binary map consisting of aligned hazard points
        GridMap composite;
    }

    static final public class Slice
    {
        int idx; // index in graph (and original index from sweep)

        double xyt[]; // where was robot when slice was taken?
        double xyt_refined[];

        double dxyt[]; // position with respect to slice 0.
        double dxyt_refined[]; // position with respect to slice 0, after alignment.

        AX12PoseLidar.Data data;
        ArrayList<double[]> pointsBody; // robot coordinates
        ArrayList<double[]> pointsLocal;

        ArrayList<double[]> slamPointsBody = new ArrayList<double[]>();
    }

    // decreasing order of number of vertical points
    public static class SlicesByAngle implements Comparator<Slice>
    {
        public int compare(Slice a, Slice b)
        {
            double va = Math.abs(a.data.status.position_radians);
            double vb = Math.abs(b.data.status.position_radians);

            return Double.compare(va, vb);
        }
    }

    public interface Listener
    {
        public void handle(ArrayList<Sweep> sweeps, Sweep sweep);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("CMDS_"+RobotUtil.getGNDID())) {
                robot_command_list_t commandList = new robot_command_list_t(ins);
                if (commandList == null || (ensureMonotonicMsgs && cmdsUtime >= commandList.utime))
                    return;
                cmdsUtime = commandList.utime;
                cmd_list = commandList;
            }
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }

    public TerrainMapper(Config config, GetOpt gopt)
    {
        ensureMonotonicMsgs = config.getBoolean("lcm.accept_only_monotonic_utimes", true);

        this.config = config;
        this.gopt = gopt;
        apl = new AX12PoseLidar(5);
        apl.addListener(this);

        lcm.subscribe("CMDS_"+RobotUtil.getGNDID(), this);

        gpsParser = new GarminNMEAParser();
        gpsParser.addListener(this);

        new WorkerThread().start();
        if (gopt.getBoolean("gui")) {
            Tester gui = new Tester(config, gopt);
            this.addListener(gui);
        }

    }

    public void notifyGPS(gps_t gps)
    {
        gpsCache.put(gps, gps.host_utime);
    }

    public void addListener(Listener listener)
    {
        listeners.add(listener);
    }

    public void handleData(AX12PoseLidar.Data d)
    {
    }

    public void handleSweep(ArrayList<AX12PoseLidar.Data> datas)
    {
        synchronized (datasQueueObject) {
            if (datasQueue != null)
                System.out.println("Dropping sweep");

            datasQueue = datas;
            datasQueueObject.notifyAll();
        }
    }

    class WorkerThread extends Thread
    {
        public void run()
        {
            while (true) {
                ArrayList<AX12PoseLidar.Data> datas;

                synchronized (datasQueueObject) {
                    datas = datasQueue;
                    datasQueue = null;

                    if (datas == null) {
                        try {
                            datasQueueObject.wait();
                        } catch (InterruptedException ex) {
                        }
                    }
                }

                if (datas != null) {
                    try {
                        handleSweepReal(datas);
                    } catch (Exception ex) {
                        System.out.println("ex: "+ex);
                        ex.printStackTrace();
                        System.exit(1);
                    }
                }
            }
        }
    }

    void handleSweepReal(ArrayList<AX12PoseLidar.Data> datas)
    {
        // drop incomplete sweeps.
        if (datas.size() < min_slices)
            return;

        Sweep sweep = new Sweep();
        sweeps.add(sweep);
        while (sweeps.size() > max_sweeps)
            sweeps.remove(0);

        // free unneeeded crap.
        for (int sweepidx = 0; sweepidx < sweeps.size() + LinAlg.min(hazard_history) - 1; sweepidx++) {
            Sweep s = sweeps.get(sweepidx);
            s.slices = null;
            s.ground = null;
            s.slamgm = null;
            s.slamgm_blurred = null;
            s.aligngm = null;
            s.hazardgm = null;
            s.composite = null;
        }

        double range = 35;
        double ground_model_range = 20;

        // pick an arbitrary pose for initializing ranges
        pose_t pose = datas.get(0).pose;

        sweep.ground = new GroundModel(pose.pos[0]-ground_model_range, pose.pos[1]-ground_model_range,
                                       2*ground_model_range, 2*ground_model_range, 0.25, ground_slope);

        VoxelModel voxels = new VoxelModel(pose.pos[0]-range, pose.pos[1]-range,
                                           2*range, 2*range, 0.125, pose.pos[2]-3, 0.1);

        /////////////////////////////////////////////////////
        // Create slices, and project lidar points. Build ground
        // model.
        for (int sidx = 0; sidx < datas.size(); sidx++) {
            Slice slice = new Slice();
            slice.idx = sidx;
            slice.data = datas.get(sidx);

            sweep.slices.add(slice);

            // body to local frame
            double B2L[][] = LinAlg.quatPosToMatrix(slice.data.pose.orientation, slice.data.pose.pos);
            slice.xyt = LinAlg.matrixToXYT(B2L);

            // sensor to body
            double S2B[][] = LinAlg.multiplyMany(ConfigUtil.getRigidBodyTransform(config, "HOKUYO_LIDAR"),
                                                 LinAlg.rotateY(-slice.data.status.position_radians),
                                                 LinAlg.translate(0,0,config.requireDouble("HOKUYO_LIDAR.zoffset")));

            ArrayList<double[]> pointsRaw = new ArrayList<double[]>();

            for (int i = 0; i < slice.data.laser.nranges; i++) {
                double r = slice.data.laser.ranges[i];

                // Skip points with error codes
                if (r < 0)
                    continue;

                double theta = slice.data.laser.rad0 + slice.data.laser.radstep*i;

                double xyz[] = new double[3];
                xyz[0] = r*Math.cos(theta);
                xyz[1] = r*Math.sin(theta);
                pointsRaw.add(xyz);
            }

            // filter out points that could be hallucinated
            if (config.requireBoolean("HOKUYO_LIDAR.filter.enable"))
                pointsRaw = GlancingPointFilter.filter(pointsRaw,
                                                       config.requireDouble("HOKUYO_LIDAR.filter.minDistance"),
                                                       config.requireDouble("HOKUYO_LIDAR.filter.minProjection"));

// this is actually wrong: we require that we get the right points when we project by xyt
//            slice.pointsBody = LinAlg.transform(S2B, pointsRaw);

            // sensor to local frame
            double S2L[][] = LinAlg.multiplyMany(B2L, S2B);

            slice.pointsLocal = LinAlg.transform(S2L, pointsRaw);
            slice.pointsBody = LinAlg.transform(LinAlg.xytInverse(slice.xyt), slice.pointsLocal);

            // we're building a ground model on the unaligned points,
            // but our alignment problems won't mess with things that much!
            for (double p[] : slice.pointsLocal) {
                sweep.ground.addPoint(p[0], p[1], p[2]);
                voxels.addPoint(p[0], p[1], p[2]);
            }
        }

        Collections.sort(sweep.slices, new SlicesByAngle());

        // For this sweep, make the canonical xyt equal to the most horizontal slice.
        if (true) {
            Slice slice = sweep.slices.get(0);

            sweep.utime = slice.data.laser.utime;
            sweep.xyt = slice.xyt;
        }

        // generate slammable surfaces map
        if (true) {

            // apply SLAM height / vertical support critera to each slice.
            for (int sliceidx = 0; sliceidx < sweep.slices.size(); sliceidx++) {

                Slice slice = sweep.slices.get(sliceidx);

                for (int pidx = 0; pidx < slice.pointsLocal.size(); pidx++) {
                    double lp[] = slice.pointsLocal.get(pidx);

                    double groundz = sweep.ground.getGroundHeight(lp[0], lp[1]);
                    double height = lp[2] - groundz;
                    if (height < slam_zmin || height > slam_zmax)
                        continue;

                    double unsupported = voxels.getUnsupportedHeight(lp[0], lp[1], groundz+0.3, lp[2]);
                    if (unsupported >= slam_max_unsupported)
                        continue;

                    // XXX check that more than one slice observed something here.

                    slice.slamPointsBody.add(slice.pointsBody.get(pidx));
                }
            }

            // align slices by their slam points. We build this in the body frame of slice[0].

            // slamgm: a binary occupancy grid of aligned vertical surfaces
            sweep.slamgm = GridMap.makeMeters(pose.pos[0]-range, pose.pos[1]-range,
                                              2*range, 2*range, 0.05, 0);

            AssignmentModel assignment = new AssignmentModel(pose.pos[0]-range, pose.pos[1]-range,
                                                             2*range, 2*range, 0.2);


            // aligngm: a blurred version of slam surfaces, built
            // incrementally, used to help align slices.
            sweep.aligngm = sweep.slamgm.copy();
            GridMap.LUT lut = sweep.aligngm.makeGaussianLUT(1.0, 0, 1.0 / LinAlg.sq(0.06));

            double last_lp[] = null;
            int lastsliceidx = -1;
            int lastpidx = -1;

            MultiResolutionScanMatcher alignmatcher = new MultiResolutionScanMatcher(new Config());
            alignmatcher.decimate = 1; // disable, since we only use refinement.

            for (int sliceidx = 0; sliceidx < sweep.slices.size(); sliceidx++) {

                Slice slice = sweep.slices.get(sliceidx);

                if (sliceidx == 0) {
                    // take first slice as "truth"
                    slice.xyt_refined = slice.xyt;
                } else {
                    // align to previous sweeps
                    alignmatcher.setModel(sweep.aligngm);

                    double P[][] = LinAlg.diag(new double[] { LinAlg.sq(0.15),
                                                              LinAlg.sq(0.15),
                                                              LinAlg.sq(Math.toRadians(10)) });
                    double priorScale = 1.0 / (slice.slamPointsBody.size() * 100.0);
                    double priorScaled[][] = LinAlg.scale(P, priorScale);

                    slice.xyt_refined = alignmatcher.refine(slice.slamPointsBody,
                                                            slice.xyt[0], slice.xyt[1], slice.xyt[2],
                                                            slice.xyt, LinAlg.inverse(priorScaled));

                }

                slice.dxyt = LinAlg.xytInvMul31(sweep.xyt, slice.xyt);
                slice.dxyt_refined = LinAlg.xytInvMul31(sweep.xyt, slice.xyt_refined);

                // now we've aligned the points in this scan. Add them
                // to the grid map (using our assignment scheme).
                ArrayList<double[]> alignedPointsLocal = LinAlg.transform(slice.xyt_refined,
                                                                          slice.slamPointsBody);

                ArrayList<double[]> alignedPointsBody = LinAlg.transform(LinAlg.xytInverse(sweep.xyt),
                                                                         alignedPointsLocal);

                for (int pidx = 0; pidx < alignedPointsLocal.size(); pidx++) {

                    double lp[] = alignedPointsLocal.get(pidx);

                    // if this region has already been assigned to
                    // another slice, we skip over this point.
                    int assn = assignment.getAssignment(lp[0], lp[1]);
                    if (assn != -1 && assn != sliceidx)
                        continue;

                    // assign this area to our slice
                    if (assn == -1)
                        assignment.setAssignment(lp[0], lp[1], sliceidx);

                    sweep.slamPointsBody.add(alignedPointsBody.get(pidx));

                    sweep.slamgm.setValue(lp[0], lp[1], (byte) 255);
                    sweep.aligngm.drawDot(lp[0], lp[1], lut);

                    if (last_lp != null && lastsliceidx==sliceidx && lastpidx == pidx-1) {
                        double dist = LinAlg.distance(lp, last_lp);
                        if (dist < 0.25) {
                            sweep.slamgm.drawLine(last_lp[0], last_lp[1], lp[0], lp[1], (byte) 255);
                            // aligngm.drawLine(lastgp[0], lastgp[1], gp[0], gp[1], lut);
                        }
                    }

                    last_lp = lp;
                    lastsliceidx = sliceidx;
                    lastpidx = pidx;
                }
            }

            sweep.slamgm = sweep.slamgm.crop(true);

            if (true) {
                // generate blurred map for matching
                sweep.slamgm_blurred = sweep.slamgm.copy();
                float f[] = SigProc.makeGaussianFilter(1.5, 5);
                f = LinAlg.scale(f, 1.0 / LinAlg.max(f));

                sweep.slamgm_blurred.filterFactoredCenteredMax(f, f);
            }

            /////////////////////////////////////////////////////////////////////////////
            // Match this slammap against the previous.
            if (sweeps.size() == 1) {
                sweep.xyt_slam = LinAlg.copy(sweep.xyt);
            } else {
                Sweep sweepa = sweeps.get(sweeps.size()-2);
                GridMap gma = sweepa.slamgm_blurred;
                double xyta[] = sweepa.xyt;

                // sweepb is the current sweep. But we'll call it
                // sweepb to be consistent with other scan matching
                // applications.
                Sweep sweepb = sweeps.get(sweeps.size()-1);
                assert(sweepb == sweep);

                ArrayList<double[]> pointsb = sweepb.slamPointsBody;
                double xytb[] = sweepb.xyt;

                MultiResolutionMatcher matcher = new MultiResolutionMatcher();
                matcher.setModel(gma);

                double xyrange = 1.5;
                double trange = Math.toRadians(10);
                double dxyt_prior[] = LinAlg.xytInvMul31(xyta, xytb);

                double P[][] = LinAlg.diag(new double[] { LinAlg.sq(0.15+0.1*Math.abs(dxyt_prior[0])),
                                                          LinAlg.sq(0.15+0.1*Math.abs(dxyt_prior[1])),
                                                          LinAlg.sq(Math.toRadians(15) + 0.1*Math.abs(MathUtil.mod2pi(dxyt_prior[2]))) });
                double priorScale = 1.0 / (pointsb.size()*50.0);
                double priorScaled[][] = LinAlg.scale(P, priorScale);

                double res[] = matcher.match(pointsb, xytb, LinAlg.inverse(priorScaled),
                                             xytb,
                                             xyrange, xyrange, trange, Math.toRadians(1));

                if (true) {
                    HillClimbing hc = new HillClimbing(new Config());
                    hc.setModel(gma);
                    res = hc.match(pointsb, xytb, LinAlg.inverse(priorScaled),
                                   LinAlg.copy(res, 3));
                }

                double xytb_posterior[] = LinAlg.copy(res, 3);
                double dxyt_posterior[] = LinAlg.xytInvMul31(xyta, xytb_posterior);

                double err[] = LinAlg.subtract(xytb_posterior, xytb);
                err[2] = MathUtil.mod2pi(err[2]);

                double err_dist = Math.sqrt(LinAlg.sq(err[0]) + LinAlg.sq(err[1]));
                double err_t = Math.abs(MathUtil.mod2pi(err[2]));

                if (err_dist > 0.5 || err_t > Math.toRadians(15)) {
                    sweep.xyt_slam = LinAlg.xytMultiply(sweepa.xyt_slam, dxyt_prior);
                    System.out.println("rejecting slam");
                } else {
                    sweep.xyt_slam = LinAlg.xytMultiply(sweepa.xyt_slam, dxyt_posterior);
                }
            }

            // experimental hazard map
            if (true) {
                Tic tic = new Tic();

                HazardMap hm;

                if (config.requireBoolean("obstacle.use_flat_world")) {
                    hm = new FlatWorldHazardMap(pose.pos[0] - hazard_range, pose.pos[1] - hazard_range,
                                                2*hazard_range, 2*hazard_range, 0.05,
                                                maxJump, obstacleHeight, niters,
                                                pose.pos[2], sweep, gopt.getBoolean("gui") ? vw : null);
                } else {
                    // Standard MAGIC "up" detector
                    hm = new MagicHazardMap(pose.pos[0] - hazard_range, pose.pos[1] - hazard_range,
                                            2*hazard_range, 2*hazard_range, 0.05,
                                            maxJump, obstacleHeight, niters);
                }



                // identify the hazard points
                for (int sliceidx = 0; sliceidx < sweep.slices.size(); sliceidx++) {

                    Slice slice = sweep.slices.get(sliceidx);

                    ArrayList<double[]> pointsRefinedLocal = LinAlg.transform(slice.xyt_refined, slice.pointsBody);

                    for (int pidx = 0; pidx < pointsRefinedLocal.size(); pidx++) {
                        double lp[] = pointsRefinedLocal.get(pidx);

                        // XXX reject points safely overhead
                        if (!config.requireBoolean("obstacle.use_flat_world")) {
                            double groundz = sweep.ground.getGroundHeight(lp[0], lp[1]);
                            double height = lp[2] - groundz;

                            if (height < hazard_zmin || height > hazard_zmax)
                                continue;
                            //if (height > hazard_zmax)
                            //    continue;
                        }

                        hm.addPoint(lp[0], lp[1], lp[2]);
                    }
                }

                hm.compute();

                sweep.hazardgm = hm.makeGridMap();

                System.out.printf("hazard gen time: %.2f ms\n", tic.toc()*1000.0);
            }

            /////////////////////////////////////////////////////////////////////////////
            // publish hazard map. We composite in the local
            // coordinate frame (NOT the slam frame) but make use of
            // the SLAM corrections.
            if (true) {
                // hazard map: the map for THIS sweep.
                /*
                sweep.hazardgm = GridMap.makeMeters(pose.pos[0] - hazard_range, pose.pos[1] - hazard_range,
                                                    2*hazard_range, 2*hazard_range, 0.05, 0);

                // identify the hazard points
                for (int sliceidx = 0; sliceidx < sweep.slices.size(); sliceidx++) {

                    Slice slice = sweep.slices.get(sliceidx);

                    ArrayList<double[]> pointsRefinedLocal = LinAlg.transform(slice.xyt_refined, slice.pointsBody);

                    for (int pidx = 0; pidx < pointsRefinedLocal.size(); pidx++) {
                        double lp[] = pointsRefinedLocal.get(pidx);

                        // XXX reject points safely overhead
                        double groundz = sweep.ground.getGroundHeight(lp[0], lp[1]);
                        double height = lp[2] - groundz;

                        if (height < hazard_zmin || height > hazard_zmax)
                            continue;

                        double unsupported = voxels.getUnsupportedHeight(lp[0], lp[1], groundz+0.3, lp[2]);
                        if (unsupported >= hazard_max_unsupported)
                            continue;

                        sweep.hazardgm.setValue(lp[0], lp[1], (byte) 255);
                    }
                }
                */

                //removed -JS sweep.hazardgm = sweep.hazardgm.crop(true);

                final int EMPTY_SPACE = 150;

                GridMap visibilityMap = GridMapRenderer.makeVisibilityMap(sweep.hazardgm, sweep.xyt);
                assert(visibilityMap.data.length == sweep.hazardgm.data.length);
                for (int i = 0; i < visibilityMap.data.length; i++) {
                    if (visibilityMap.data[i] != 0 && sweep.hazardgm.data[i]==0)
                        sweep.hazardgm.data[i] = (byte) EMPTY_SPACE;
                }

                // 0: unknown
                // EMPTY_SPACE: known empty
                // 255: occupied

                GridMap composite = GridMap.makeMeters(pose.pos[0]-hazard_range, pose.pos[1]-hazard_range,
                                                       hazard_range*2, hazard_range*2, 0.05, 0);

                for (int hhidx = 0; hhidx < hazard_history.length; hhidx++) {
                    int sweepidx = sweeps.size() - 1 + hazard_history[hhidx];
                    if (sweepidx < 0)
                        continue;

                    Sweep sw = sweeps.get(sweepidx);

                    double destxyt[] = sweeps.get(sweeps.size()-1).xyt;
                    double destxyt_slam[] = sweeps.get(sweeps.size()-1).xyt_slam;

                    double srcxyt[] = sw.xyt;
                    double srcxyt_slam[] = sw.xyt_slam;

                    // project from composite into src.
                    // srcxyt * inv(srcxyt_slam) * destxyt_slam * inv(dest)
                    double dxyt[] = LinAlg.xytMultiply(srcxyt, LinAlg.xytInverse(srcxyt_slam));
                    dxyt = LinAlg.xytMultiply(dxyt, destxyt_slam);
                    dxyt = LinAlg.xytMultiply(dxyt, LinAlg.xytInverse(destxyt));

                    double c = Math.cos(dxyt[2]), s = Math.sin(dxyt[2]);

                    for (int iy = 0; iy < composite.height; iy++) {
                        for (int ix = 0; ix < composite.width; ix++) {
                            int oldv = composite.data[iy*composite.width+ix] & 0xff;

                            double _x = composite.x0 + ix*composite.metersPerPixel;
                            double _y = composite.y0 + iy*composite.metersPerPixel;

                            // Adjust x,y per SLAM results
                            double x = _x*c - _y*s + dxyt[0];
                            double y = _x*s + _y*c + dxyt[1];

                            int v = sw.hazardgm.getValue(x, y) & 0xff;

                            if (v == 0) // no data in the new map
                                continue;

                            int newv = oldv;

                            if (oldv == 0)
                                newv = v;

                            // the most recent sweep is always right about obstacles.
                            if (oldv == 255 && v == EMPTY_SPACE)
                                newv = EMPTY_SPACE;

                            if (hazard_history[hhidx] == 0 && v == 255)
                                newv = 255;

                            composite.data[iy*composite.width+ix] = (byte) newv;
                        }
                    }
                }

                if (true) {
                    // Intelligent crop of sweep: leave a 4x4 meter bounding box + any nonzero
                    int rx = (int) ((sweep.xyt[0] - composite.x0) / composite.metersPerPixel);
                    int ry = (int) ((sweep.xyt[1] - composite.y0) / composite.metersPerPixel);
                    int xmin = (int)(rx - 4.0 /composite.metersPerPixel);
                    int ymin = (int)(ry - 4.0 /composite.metersPerPixel);

                    int xmax = (int)(rx + 4.0 /composite.metersPerPixel);
                    int ymax = (int)(ry + 4.0 /composite.metersPerPixel);

                    for (int y =0; y < composite.height; y++) {
                        for (int x =0; x < composite.width; x++) {
                            if (composite.data[y*composite.width +x] != 0) {
                                xmin = Math.min(xmin, x);
                                xmax = Math.max(xmax, x);
                                ymin = Math.min(ymin, y);
                                ymax = Math.max(ymax, y);
                            }
                        }
                    }
                    sweep.composite = composite.resizePixels(xmin,ymin, xmax-xmin + 1,ymax-ymin +1, true);
                }

                // XXX: Must be sent in local (pose) coordinates.
                lcm.publish("TERRAIN_MAP", GridMapUtil.encodeGZIP(GridMapUtil.gridMap_to_grid_map_t(sweep.composite, sweep.utime,
                                                                                                    grid_map_t.ENCODING_NONE)));
            }

            /////////////////////////////////////////////////////////////////////////////
            // Build terrain map to send over the xtend radio.
            if (true) {
                //GridMap src = sweep.slamgm.decimateMax(2);
                GridMap src = sweep.slamgm.copy();  // XXX WSF Hack

                try {
                    int maxlength = 800;

                    while (true) {
                        // get rid of extranneous space.
                        GridMap crop = src.crop(true);

                        GridMapModel amodel = new GridMapModel(crop.width, crop.height);
                        byte adata[] = ArithCode.encode(amodel, crop.data);
                        byte zdata[] = ZUtil.compress(crop.data);

                        grid_map_tz gmz = new grid_map_tz();
                        gmz.xy = new float[] { (float) crop.x0,
                                               (float) crop.y0 };
                        gmz.meters_per_pixel = (float) crop.metersPerPixel;
                        gmz.width = (short) crop.width;
                        gmz.height = (short) crop.height;

                        if (gmz.data == null || zdata.length < gmz.datalen) {
                            gmz.encoding = grid_map_t.ENCODING_GZIP;
                            gmz.datalen = (short) zdata.length;
                            gmz.data = zdata;
                        }

                        if (gmz.data == null || adata.length < gmz.datalen) {
                            gmz.encoding = grid_map_t.ENCODING_ARITH;
                            gmz.datalen = (short) adata.length;
                            gmz.data = adata;

                            if (false) {
                                // verify encoding?
                                amodel = new GridMapModel(crop.width, crop.height);
                                byte adataout[] = ArithCode.decode(amodel, adata);
                                assert(Arrays.equals(adataout, crop.data));
                            }
                        }

                        // add other encodings here.

                        // if message is too big, crop the image some and try again.
                        if (gmz.datalen > maxlength) {
                            double newwidth = src.width * src.metersPerPixel - 8.0;
                            double newheight = src.height * src.metersPerPixel - 8.0;

                            // oh hell!
                            if (newwidth < 0 || newheight < 0) {
                                System.out.println("Can't encode terrain map!");
                                break;
                            }

                            src = src.cropMeters(pose.pos[0]-newwidth/2, pose.pos[1]-newheight/2,
                                                 newwidth, newheight, true);
                            continue;
                        }

                        if (gopt.getBoolean("train")) {
                            if (train == null) {
                                train = new GridMapModel.Train();
                                trainsizes = new int[3];
                            }

                            train.update(src.data, src.width, src.height);
                            trainsizes[1] += zdata.length;
                            trainsizes[2] += adata.length;
                            trainsizes[0] ++;
                            train.print();

                            System.out.printf("%15f %15f\n", (1.0*trainsizes[1] / trainsizes[0]),
                                              (1.0*trainsizes[2] / trainsizes[0]));
                        }


                        // message is okay! Send it!

                        robot_map_data_tz data = new robot_map_data_tz();
                        gps_t gps = gpsCache.get();
                        if (gps == null) {
                            gps = new gps_t();
                            gps.horiz_dop = 100.0f;
                            gps.nsats = 0;
                        }
                        data.gps = TZUtil.encode(gps);
                        data.gridmap = gmz;

                        data.xyt_local = TZUtil.encode(sweep.xyt);
                        data.xyt_slam = TZUtil.encode(sweep.xyt_slam);
                        data.utime = sweep.utime;
                        lcm.publish(gopt.getString("channel") + "_TX", data);
                        lcm.publish(gopt.getString("channel") + "_LTX", data);

                        break;
                    }
                } catch (IOException ex) {
                    System.out.println("ex: "+ex);
                }
            }
        }

        /////////////////////////////////////////////////////////
        // Publish slam points by slice for PedTracker
        slam_points_sweep_t SPSweep = new slam_points_sweep_t();
        SPSweep.utime   = sweep.utime;
        SPSweep.xyt     = sweep.xyt; // XXX could use xyt_slam here, but it drifts at high replay rates
        SPSweep.nslices = sweep.slices.size();
        SPSweep.slices  = new slam_points_slice_t[sweep.slices.size()];
        for (int sliceidx = 0; sliceidx < sweep.slices.size(); sliceidx++) {

            Slice slice = sweep.slices.get(sliceidx);

            SPSweep.slices[sliceidx]                    = new slam_points_slice_t();
            SPSweep.slices[sliceidx].original_sweep_idx = slice.idx;
            SPSweep.slices[sliceidx].xyt                = slice.xyt_refined;

            ArrayList<double[]> refinedPoints           = LinAlg.transform(slice.xyt_refined,
                                                                           slice.slamPointsBody);

            int npoints                                 = refinedPoints.size();
            SPSweep.slices[sliceidx].npoints            = npoints;
            SPSweep.slices[sliceidx].local_points       = new double[npoints][];

            for (int i=0; i < npoints; i++)
                SPSweep.slices[sliceidx].local_points[i] = refinedPoints.get(i);
        }

        lcm.publish("SLAM_POINTS_SWEEP", SPSweep);

        /////////////////////////////////////////////////////////
        // All done, publish!
        for (Listener listener : listeners)
            listener.handle(sweeps, sweep);

    }

    static double[] poseToXyt(pose_t p)
    {
        return new double[] { p.pos[0], p.pos[1], LinAlg.quatToRollPitchYaw(p.orientation)[2] };
    }

    class Tester extends Thread implements TerrainMapper.Listener
    {
        JFrame jf;
        //VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
VisCanvas vc = new VisCanvas(vl);
        Config config;
        ParameterGUI pg;

        ArrayList<Sweep> sweeps;
        Sweep sweep;

        public Tester(Config config, GetOpt gopt)
        {
            this.config = config;
            jf = new JFrame("TerrainMapper");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.add(vc, BorderLayout.CENTER);
            jf.add(new LayerBufferPanel(vc), BorderLayout.EAST);
            jf.setSize(800,600);
            jf.setVisible(true);

            vw.getBuffer("grid").addFront(new VzGrid());
            VisCameraManager.CameraPosition pos = vl.cameraManager.getCameraTarget();
            pos.perspectiveness = 0;
            vl.cameraManager.goUI(pos);

//            vc.setTargetFPS(5);

            //TerrainMapper salign = new TerrainMapper(config, gopt);
            //salign.addListener(this);

            vl.addEventHandler(new april.viewer.RemoteLogEventHandler(vl, null, null));
            pg = new ParameterGUI();
            pg.addDoubleSlider("height", "Obstacle Height", 0, 1.0, obstacleHeight);
            pg.addDoubleSlider("maxJump", "Max Neighbor Jump", 0, 10.0, maxJump);
            pg.addIntSlider("niters", "Iterations", 0, 10, niters);
            pg.addListener(new ParameterListener() {
                    public void parameterChanged(ParameterGUI pg, String name)
                    {
                        obstacleHeight = pg.gd("height");
                        maxJump = pg.gd("maxJump");
                        niters = pg.gi("niters");
                    }
                });
            jf.add(pg, BorderLayout.SOUTH);
            this.start();
        }

        public void run()
        {
            ArrayList<Sweep> sweeps;
            Sweep sweep;

            while(true) {
                TimeUtil.sleep(250);
                synchronized(this) {
                    sweeps = this.sweeps;
                    sweep = this.sweep;
                }
                draw(sweeps, sweep);
            }

        }

        public void handle(ArrayList<Sweep> sweeps, Sweep sweep)
        {
            synchronized(this) {
                this.sweeps = sweeps;
                this.sweep = sweep;
            }
        }

        public void draw(ArrayList<Sweep> sweeps, Sweep sweep)
        {
            if (sweeps == null || sweep == null)
                return;
            if (true) {
                VisWorld.Buffer vb = vw.getBuffer("robots");
                vb.setDrawOrder(100);
                for (Slice slice : sweep.slices)
                    vb.addBack(new VisDepthTest(false, new VisChain(LinAlg.xytToMatrix(slice.xyt),
                                                                        new VzRobot())));

                vb.swap();
            }

            if (true) {
                VisWorld.Buffer vb = vw.getBuffer("points-unaligned");
                double ssMin = Double.MAX_VALUE;
                double ssMax = -Double.MAX_VALUE;
                for (TerrainMapper.Slice slice : sweep.slices) {
                    vb.addBack(new VisChain(LinAlg.xytToMatrix(sweep.xyt),
                                            LinAlg.xytToMatrix(slice.dxyt),
                                            new VzPoints(new VisVertexData(slice.pointsBody),
                                                         new VzPoints.Style(Color.green, 2))));
                }

                vb.swap();
            }

            if (false) {
                VisWorld.Buffer vb = vw.getBuffer("slam points-aligned");
                for (TerrainMapper.Slice slice : sweep.slices) {

                    vb.addBack(new VisChain(LinAlg.xytToMatrix(sweep.xyt),
                                            LinAlg.xytToMatrix(slice.dxyt_refined),
                                            new VzPoints(new VisVertexData(slice.slamPointsBody),
                                                         new VzPoints.Style(Color.blue, 1))));
                }

                vb.swap();
            }



            if (true) {
                VisWorld.Buffer vb = vw.getBuffer("slam: gridmap");
                vb.setDrawOrder(1);
                GridMap gm = sweep.slamgm;
                double vertices[][] = {{gm.x0, gm.y0},
                                       {gm.x0, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0}};
                double tiepoints[][] = {{0,0}, {0,1}, {1,1},{1,0}};

                vb.addBack(new VisChain(new VzImage(new VisTexture(gm.makeBufferedImage()),
                                                    vertices, tiepoints, null)));
                vb.swap();
            }

            if (false) {
                VisWorld.Buffer vb = vw.getBuffer("align: gridmap");
                vb.setDrawOrder(1);
                GridMap gm = sweep.aligngm;
                double vertices[][] = {{gm.x0, gm.y0},
                                       {gm.x0, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0}};
                double tiepoints[][] = {{0,0}, {0,1}, {1,1},{1,0}};
                vb.addBack(new VisChain(new VzImage(new VisTexture(gm.makeBufferedImage()),
                                                    vertices, tiepoints, null)));
                vb.swap();
            }

            if (false) {
                VisWorld.Buffer vb = vw.getBuffer("slam: blurred gridmap");
                vb.setDrawOrder(1);
                GridMap gm = sweep.slamgm_blurred;
                double vertices[][] = {{gm.x0, gm.y0},
                                       {gm.x0, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0}};
                double tiepoints[][] = {{0,0}, {0,1}, {1,1},{1,0}};
                vb.addBack(new VisLighting(false,
                                           new VzImage(new VisTexture(gm.makeBufferedImage()),
                                                       vertices, tiepoints, null)));
                vb.swap();
            }

            if (true) {
                VisWorld.Buffer vb = vw.getBuffer("composite hazardmap");
                vb.setDrawOrder(1);
                GridMap gm = sweep.composite;
                double vertices[][] = {{gm.x0, gm.y0},
                                       {gm.x0, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0}};
                double tiepoints[][] = {{0,0}, {0,1}, {1,1},{1,0}};
                vb.addBack(new VisLighting(false,
                                           new VzImage(new VisTexture(gm.makeBufferedImage()),
                                                       vertices, tiepoints, null)));
                vb.swap();
            }

            if (true) {
                VisWorld.Buffer vb = vw.getBuffer("hazardmap");
                vb.setDrawOrder(1);
                GridMap gm = sweep.hazardgm;
                double vertices[][] = {{gm.x0, gm.y0},
                                       {gm.x0, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0 + gm.height*gm.metersPerPixel },
                                       {gm.x0 + gm.width*gm.metersPerPixel, gm.y0}};
                double tiepoints[][] = {{0,0}, {0,1}, {1,1},{1,0}};
                vb.addBack(new VisLighting(false,
                                           new VzImage(new VisTexture(gm.makeBufferedImage()),
                                                       vertices, tiepoints, null)));
                vb.swap();
            }

            if (false) {
                VisWorld.Buffer vb = vw.getBuffer("hist-points-slam");
                for (Sweep s : sweeps) {
                    Color c = ColorUtil.seededColor((int) s.utime);
                    vb.addBack(new VisChain(LinAlg.xytToMatrix(s.xyt_slam),
                                            new VzRobot(c),
                                            new VzPoints(new VisVertexData(s.slamPointsBody),
                                                         new VzPoints.Style(c, 1))));
                }
                vb.swap();
            }

            if (true) {
                pose_t pose = PoseTracker.getSingleton().get();

                VisWorld.Buffer vb = vw.getBuffer("ground");
                GroundModel ground = sweep.ground;
                // Vis2: Use VzMesh -- need to generate points and index vector
                vb.addBack(new VzGround(ground, ColorMapper.makeJet(pose.pos[2]-1, pose.pos[2]+1)));
                // vb.addBack(new VisChain(new VisMesh(ground.x0, ground.y0, ground.metersPerPixel, ground.metersPerPixel,
                //                                         ground.width, ground.height, ground.data, ColorMapper.makeJet(pose.pos[2]-1, pose.pos[2]+1))));
                vb.swap();
            }

        }
    }

    public static void main(String args[])
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('g', "gui", false, "Show GUI");
        gopt.addBoolean('h', "help", false, "Show this help");
        gopt.addBoolean('\0', "train", false, "Train ArithCode model");
        gopt.addString('c', "channel", "ROBOT_MAP_DATA", "Channel for robot_map_data_t (TX and LTX added)");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            gopt.doHelp();
            return;
        }

        new TerrainMapper(RobotUtil.getConfig(), gopt);

        while (true) {
            TimeUtil.sleep(1000);
        }
    }

}
