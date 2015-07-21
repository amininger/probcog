package probcog.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.vis.*;
import april.util.*;

import probcog.vis.*;
import probcog.classify.*;
import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.util.*;
import probcog.navigation.*;

import probcog.lcmtypes.*;
import magic2.lcmtypes.*;

public class MonteCarloBot implements SimObject
{
    private static final double GM_SIZE_M = 8.0;
    private static final double GM_MPP = 0.05;

    Random r = new Random();

    int ROBOT_ID = 6;
    SimWorld sw;

    Object runLock = new Object();
    boolean running = false;

    FastDrive drive = null;
    double initialDistanceTraveled = 0;
    ArrayList<double[]> trajectoryTruth = new ArrayList<double[]>();
    ArrayList<double[]> trajectoryOdom = new ArrayList<double[]>();
    VisColorData vcd = new VisColorData();

    TagClassifier tc;

    // Control law/condition test. Need to be cast appropriately for use...
    ControlLaw law;
    ConditionTest test;

    // Internal Simulation State
    int simSinceReset = 0;
    int iteration = 0;
    boolean success = false;        // Did the simulation terminate without timeout?

    public MonteCarloBot(SimWorld sw)
    {
        this.sw = sw;
        try {
            this.tc = new TagClassifier(false);
        } catch (IOException ioex) {
            ioex.printStackTrace();
        }

    }

    private HashMap<String, LabelCountRecord> counts =
        new HashMap<String, LabelCountRecord>();
    private class LabelCountRecord
    {
        public int count = 0;
        public double prob = 1.0;
        public double myprob = 1.0;
    }

    private HashMap<String, ClassificationCounterTest> testMap =
        new HashMap<String, ClassificationCounterTest>();
    public HashMap<Behavior, Behavior> tagRecords = new HashMap<Behavior, Behavior>();

    // For use in multi-simulation tag observations
    public ArrayList<classification_t> observations = new ArrayList<classification_t>();

    // === Random sampling interface =========================
    private void resetTrajectories()
    {
        simSinceReset = 0;
        vcd = new VisColorData();
        trajectoryTruth.clear();
        trajectoryOdom.clear();
        testMap.clear();
        // NOTE: does not reset tagRecords.
    }

    public void init(ControlLaw law, ConditionTest test)
    {
        init(law, test, null, null, 0);
    }

    public void init(ControlLaw law,
                     ConditionTest test,
                     double[] xyt,
                     double[] odomxyt,
                     double initialDistanceTraveled)
    {
        success = false;
        this.law = law;
        this.test = test;

        if (xyt != null) {
            drive = new FastDrive(sw, this, xyt, odomxyt);
            drive.centerOfRotation = new double[] { 0.13, 0, 0 };   // Right for new robot?

            resetTrajectories();
            this.initialDistanceTraveled = initialDistanceTraveled;
            trajectoryTruth.add(drive.poseTruth.pos);
            trajectoryOdom.add(drive.poseOdom.pos);
        }
    }

    /** Simulate all steps, keeping track of trajectory, etc. Maintains
     *  some information from the past, in particular w.r.t. robot
     *  position and which tags the robot is already likely to know about.
     **/
    public void simulate()
    {
        simulate(Util.getConfig().requireDouble("monte_carlo.default_forward_search_time"), false);
    }

    public void simulate(double seconds)
    {
        simulate(seconds, false);
    }

    public void simulate(boolean perfect)
    {
        simulate(Util.getConfig().requireDouble("monte_carlo.default_forward_search_time"), perfect);
    }

    public void simulate(double seconds, boolean perfect)
    {
        long currentUtime = TimeUtil.utime();

        // Precalculated paramters
        HashSet<SimObject> ignore = new HashSet<SimObject>();
        for (SimObject so: sw.objects) {
            if (so instanceof SimRobot)
                ignore.add(so);
        }
        ignore.add(this);
        double radstep = Math.toRadians(Util.getConfig().requireDouble("monte_carlo.step_degrees"));
        double minDeg = Util.getConfig().requireDouble("monte_carlo.min_degrees");
        double maxDeg = Util.getConfig().requireDouble("monte_carlo.max_degrees");
        double maxRange = Util.getConfig().requireDouble("monte_carlo.max_range");
        double rad0 = Math.toRadians(minDeg);
        double rad1 = Math.toRadians(maxDeg);
        laser_t laser = new laser_t();
        laser.utime = currentUtime;

        // Populate a grid map with robot data
        grid_map_t gm = new grid_map_t();
        gm.utime = currentUtime;

        // Populate our knowledge of tags based on what we can see already.
        // In the future, we may actually want to work on knowing exactly what
        // tags were labeled as in recent history, but for now, it is sufficient
        // to know that they were seen.
        TagHistory tagHistory = new TagHistory();
        for (SimAprilTag tag: getSeenTags()) {
            double[] xyzrpy = LinAlg.matrixToXyzrpy(tag.getPose());
            double[] relXyzrpy = relativePose(getPose(), xyzrpy);
            ArrayList<classification_t> classies = tc.classifyTag(tag.getID(), relXyzrpy, perfect);
            tagHistory.addObservations(classies, currentUtime);
            //tagHistory.markTagObserved(tag.getID());
        }

        // XXX Can we incorporate more noise here in terms of what ACTUALLY
        // happens vs. what odometry says happens?
        //
        // While control law has not finished OR timeout, try updating
        int timeout = (int)(seconds/FastDrive.DT);
        Tic tic = new Tic();
        double time = 0;

        double[] startXYT = LinAlg.matrixToXYT(getPose());
        double startDist = getTrajectoryLength();
        while ((test == null || !test.conditionMet()) && timeout > 0) {
            tic.tic();
            // LASER UPDATE
            double[][] T_truth = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                                                        drive.poseTruth.pos),
                                                 LinAlg.translate(0.3, 0, 0.25));
            double[][] T_odom = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseOdom.orientation,
                                                                       drive.poseOdom.pos),
                                                LinAlg.translate(0.3, 0, 0.25));

            double ranges[] = Sensors.laser(sw, ignore, T_truth, (int) ((rad1-rad0)/radstep), rad0, radstep, maxRange);

            if (!perfect) {
                double mean = 0;
                double stddev = 0.10;   // Laser noise
                for (int i = 0; i < ranges.length; i++) {
                    if (ranges[i] >= maxRange || ranges[i] < 0)
                        continue;
                    ranges[i] = Math.min(maxRange, ranges[i]+r.nextGaussian()*stddev*ranges[i]);
                }
            }

            laser.nranges = ranges.length;
            laser.ranges = LinAlg.copyFloats(ranges);
            laser.rad0 = (float) rad0;
            laser.radstep = (float) radstep;

            gm.x0 = drive.poseOdom.pos[0] - GM_SIZE_M/2;
            gm.y0 = drive.poseOdom.pos[1] - GM_SIZE_M/2;
            gm.encoding = grid_map_t.ENCODING_NONE;
            gm.meters_per_pixel = GM_MPP;
            gm.width = (int)Math.ceil(GM_SIZE_M/GM_MPP);
            gm.height = gm.width;
            gm.datalen = gm.width*gm.height;
            gm.data = new byte[gm.datalen];
            for (int i = 0; i < laser.nranges; i++) {
                double r = laser.ranges[i];
                if (r < 0)
                    continue;
                double robotTheta = LinAlg.quatToRollPitchYaw(drive.poseOdom.orientation)[2];
                double t = laser.rad0 + i*laser.radstep + robotTheta;
                double x = r*Math.cos(t);
                double y = r*Math.sin(t);
                int ix = (int)Math.floor((x-gm.x0)/gm.meters_per_pixel);
                int iy = (int)Math.floor((y-gm.y0)/gm.meters_per_pixel);
                if (ix < 0 || ix >= gm.width || iy < 0 || iy >= gm.height)
                    continue;
                gm.data[iy*gm.width + ix] = 1;  // SLAMMABLE
            }

            DriveParams params = new DriveParams();
            params.dt = FastDrive.DT;
            params.laser = laser;
            params.pose = drive.poseOdom; // XXX
            params.gm = gm;


            diff_drive_t dd = new diff_drive_t();
            dd.utime = TimeUtil.utime();
            dd.left_enabled = dd.right_enabled = false;
            dd.left = dd.right = 0;

            // Special behavior for driving towards a tag. Very generous with sight
            if (law instanceof DriveTowardsTag) {
                DriveTowardsTag dtt = (DriveTowardsTag)law;
                HashSet<SimAprilTag> currentTags = getSeenTags();
                params.classy = null;
                double minDist = Double.MAX_VALUE;
                for (SimAprilTag tag: currentTags) {
                    double[] xyzrpy = LinAlg.matrixToXyzrpy(tag.getPose());
                    double[] relXyzrpy = relativePose(getPose(), xyzrpy);

                    classification_t classy = tc.classifyTag(tag.getID(), relXyzrpy).get(0);
                    classy.name = tc.correctClass(tag.getID());
                    double dist = LinAlg.magnitude(LinAlg.resize(relXyzrpy, 2));
                    if (classy.name.equals(dtt.getClassType()) && dist < minDist) {
                        params.classy = classy;
                        minDist = dist;
                    }
                }
                dd = dtt.drive(params);
            } else {
                dd = law.drive(params);
            }

            currentUtime += FastDrive.DT*1000000;
            drive.motorCommands[0] = dd.left;
            drive.motorCommands[1] = dd.right;
            drive.update();
            laser.utime = currentUtime;

            // Update condition test, if necessary
            if (test instanceof Stabilized) {
                Stabilized stable = (Stabilized)test;
                pose_t stable_pose = drive.poseTruth;
                stable_pose.utime = currentUtime;
                stable.update(stable_pose);
            }

            // Update law if necessary
            //if (law instanceof Orient) {
            //    Orient orient = (Orient)law;
            //    orient.update(drive.poseOdom);
            //}

            // Rough outline:
            //  Keep a global timestamp for simulated time. This will be
            //  potentially important for timeout-related activities like tag
            //  persistence. We have TWO modes of operation. The first is the
            //  normal tag observation. We see a tag and go: Ah! Why hello tag.
            //  Have I seen you before? Do you have a label? I suppose either
            //  way, it just works its way into the counting test.
            //
            //  Second is more interesting. This is where we do forward simulation.
            //  Currently, we detect this case by having no condition test. This
            //  is probably fine, for now. In this case, we just simulate forward
            //  until we timeout, making sure to record observations along the way
            //  THIS IS WHERE WE MAY CHANGE THINGS. Right now, we just make
            //  branches based on the sets of possible counts we witness along
            //  the way. This surely misses things if you sample insufficiently.
            //  Can we just sample large numbers of tag observations to build
            //  distributions of what might have happened that way? This DOES
            //  assume independent observations, but that could be fine. (Though
            //  it does lead to the question: why not just use our obviously known
            //  distributions that we have already? Mehh.....) We then will use
            //  this to generate potential behaviors to try executing.
            HashSet<SimAprilTag> seenTags = getSeenTags();
            for (SimAprilTag tag: seenTags) {
                double[] xyzrpy = LinAlg.matrixToXyzrpy(tag.getPose());
                double[] relXyzrpy = relativePose(getPose(), xyzrpy);
                ArrayList<classification_t> classies = tc.classifyTag(tag.getID(), relXyzrpy, perfect);

                if (classies.size() < 1)
                    continue;

                // Update the condition test, if necessary
                if (test instanceof NearTag) {
                    NearTag nt = (NearTag)test;
                    nt.processTag(classies.get(0));
                }

                // If we've seen this tag recently, don't worry about it.
                // Note that this means we're doing work for the counter test
                // here. It will only ever get a new classification added for
                // a given tag ONCE, unless we reobserve it. XXX
                double range = Math.sqrt(LinAlg.sq(relXyzrpy[0]) + LinAlg.sq(relXyzrpy[1]));
                tagHistory.addObservations(classies, currentUtime);
                //if (!tagHistory.isVisible(tag.getID(), range)) {
                //    continue;
                //}
                //tagHistory.markTagObserved(tag.getID());
                //
                ArrayList<classification_t> cs = tagHistory.getLabels(tag.getID(), relXyzrpy, currentUtime);
                if (cs.size() < 1)
                    continue;
                String label = cs.get(0).name;

                // Handle one of the two cases. Case 1) We're just navigating
                // normally! Pass off the information to the condition test.
                if (test != null) {
                    if (test instanceof ClassificationCounterTest) {
                        ClassificationCounterTest cct = (ClassificationCounterTest)test;
                        //for (classification_t classy: classies) {
                        //    cct.addSample(classy);
                        //}
                        cct.addSample(tag.getID(), label);
                    }
                } else {
                    buildBehaviors(startXYT,
                                   startDist,
                                   tag,
                                   perfect);
                }
            }

            // Visualization etc.
            trajectoryTruth.add(drive.poseTruth.pos);
            trajectoryOdom.add(drive.poseOdom.pos);
            java.util.List<Color> colors = Palette.qualitative_brewer1.listAll();
            vcd.add(ColorUtil.swapRedBlue(colors.get(simSinceReset % colors.size()).getRGB()));
            //if (simSinceReset%2 == 1)
            //    vcd.add(0xffff0000);  // BGR
            //else
            //    vcd.add(0xffff00ff);  // BGR

            // TIME UPDATE
            timeout--;
            time += tic.toc();
            //System.out.printf("\t%f [s]\n", tic.toc());
        }
        success = timeout > 0;
        simSinceReset++;
        iteration++;
        //System.out.printf("%f [s]\n", time);
    }

    private void buildBehaviors(double[] startXYT,
                                double startDist,
                                SimAprilTag tag,
                                boolean perfect)
    {
        buildBehaviors(startXYT, startDist, tag, perfect, false);
    }

    /** Take a list of classifications and our tag history to update our
     *  behavior list. Doesn't actually need any tag history, since that
     *  aspect of filtering is handled before this point.
     *
     *  @param tag  The tag observed and being converted to a landmark
     *  @param repeatLandmarks  If true, allow things like "go until nth door" for n> 1
     **/
    private void buildBehaviors(double[] startXYT,
                                double startDist,
                                SimAprilTag tag,
                                boolean perfect,
                                boolean repeatLandmarks)
    {
        // Handle tags that don't have labels OR are repeat landmarks. Note that
        // in this updated version of the function, we ONLY consider the actual
        // label of the tag, not the false label.
        String tagClass = tc.correctClass(tag.getID());
        if (tagClass == "")
            return;
        if (!repeatLandmarks && counts.containsKey(tagClass))
            return;

        // XXX Is this the best way to handle observations? At this point,
        // we might as well just look the damn number up.
        int NUM_TAG_SAMPLES = 1000;  // Not used so much, now. Save some compute
        double[] xyzrpy = LinAlg.matrixToXyzrpy(tag.getPose());
        double[] relXyzrpy = relativePose(getPose(), xyzrpy);
        HashMap<String, Integer> labelCount = new HashMap<String, Integer>();
        for (int i = 0; i < NUM_TAG_SAMPLES; i++) {
            // As done elsewhere, only use the first classy if it exists
            ArrayList<classification_t> classies = tc.classifyTag(tag.getID(),
                                                                  relXyzrpy,
                                                                  perfect);
            if (classies.size() < 1)
                return;

            // Count the number of times we successfully observe this tag. Only
            // count observations of the ACTUAL label value
            String label = classies.get(0).name;
            if (!tagClass.equals(label))
                continue;
            if (!labelCount.containsKey(label))
                labelCount.put(label, 0);
            labelCount.put(label, labelCount.get(label)+1);
        }

        for (String label: labelCount.keySet()) {
            if (label.equals(""))
                continue;
            double pct = labelCount.get(label)/(double)NUM_TAG_SAMPLES;
            // We are balancing several things here.
            // 1) Likelihood of this particular outcome (a composition of
            // different sensing likelihoods)
            // 2) Likelihood of actually executing this plan well enough that
            // these outcomes will hold.
            //
            // We can only really determine 1) here. The fact that we simulated
            // this is trusted to be a close proxy, aka, we expect all of our
            // control laws to act fairly consistently, so we may be able to
            // ignore two, in which case we just start exploring 1). Downside:
            // branching factor! This was already a risk before, but continues
            // to be here, as well. Do we dare use log likelihood, or are we OK
            // basically treating low probability outcomes as the same? Can we
            // get away with inserting an arbitrary threshold in here? How do we
            // limit the number of possible outcomes?
            if (!counts.containsKey(label))
                counts.put(label, new LabelCountRecord());
            LabelCountRecord lcr = counts.get(label);
            lcr.count++;
            lcr.prob *= pct;
            lcr.myprob = pct;
            // XXX This pct storage system is dependent on the underlying model,
            // which breaks some of our black-box assumptions. We should remove
            // it if we can, or at least determine whether or not we are
            // comfortable treating all observations as IID.

            HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
            params.put("count", new TypedValue(lcr.count));
            params.put("class", new TypedValue(label));
            params.put("no-lcm", new TypedValue(0));
            params.put("goal-tag", new TypedValue(tag.getID()));
            ClassificationCounterTest cct = new ClassificationCounterTest(params);

            double[] xyt = LinAlg.matrixToXYT(getPose());
            Behavior rec = new Behavior(startXYT,
                                        xyt,
                                        startXYT,   // Unimportant here
                                        xyt,        // Unimportant here
                                        startDist,
                                        getTrajectoryLength(),
                                        law,
                                        cct);
            rec.prob = lcr.prob;
            rec.myprob = lcr.myprob;
            rec.tagID = tag.getID();

            // Before this, we had this set up so we could represent a distribution
            // of XYTS in the outcomes. Is losing that more harmful than good?
            assert (!tagRecords.containsKey(rec));
            tagRecords.put(rec, rec);
        }
    }

    public HashSet<SimAprilTag> getSeenTags()
    {
        HashSet<SimAprilTag> seenTags = new HashSet<SimAprilTag>();

        for (SimObject so: sw.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            SimAprilTag tag = (SimAprilTag)so;
            double[] xyzrpy = LinAlg.matrixToXyzrpy(so.getPose());
            double d = LinAlg.distance(drive.poseTruth.pos, xyzrpy, 2);
            double maxRange = tc.getMaxRange(tag.getID());
            if (d > maxRange)
                continue;

            // This is good enough, but not quite realistic. Our camera will
            // determine this range in reality, at which point we'll be able
            // to sample a fake observation distance, but this protects us
            // from accidentally sampling outside our camera range.
            seenTags.add((SimAprilTag)so);
        }

        return seenTags;
    }

    // Convenience function for classification_t construction
    private double[] relativePose(double[][] A, double[] xyzrpy)
    {
        double[] xyzrpy_A = LinAlg.matrixToXyzrpy(A);
        double[] p = LinAlg.resize(xyzrpy, 3);
        p = LinAlg.transform(LinAlg.inverse(A), p);
        p = LinAlg.resize(p, 6);

        // Relative yaw difference.
        p[5] = MathUtil.mod2pi(xyzrpy[5] - xyzrpy_A[5]);

        return p;
    }

    public boolean success()
    {
        return success;
    }

    public double getTrajectoryLength()
    {
        if (trajectoryTruth.size() < 2)
            return 0;

        double length = 0;
        double[] prev = trajectoryTruth.get(0);
        for (int i = 1; i < trajectoryTruth.size(); i++) {
            double[] curr = trajectoryTruth.get(i);
            length += LinAlg.distance(curr, prev);
            prev = curr;
        }

        return initialDistanceTraveled + length;
    }

    public ArrayList<double[]> getTrajectoryTruth()
    {
        return trajectoryTruth;
    }

    public double getOdomLength()
    {
        if (trajectoryOdom.size() < 2)
            return 0;

        double length = 0;
        double[] prev = trajectoryOdom.get(0);
        for (int i = 1; i < trajectoryOdom.size(); i++) {
            double[] curr = trajectoryOdom.get(i);
            length += LinAlg.distance(curr, prev);
            prev = curr;
        }

        return initialDistanceTraveled + length;
    }

    // === SimObject interface ===============================
    public double[][] getPose()
    {
        return LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                      drive.poseTruth.pos);
    }

    public double[][] getOdom()
    {
        return LinAlg.quatPosToMatrix(drive.poseOdom.orientation,
                                      drive.poseOdom.pos);
    }

    public void setPose(double[][] T)
    {
        if (drive == null) {
            drive = new FastDrive(sw, this, LinAlg.matrixToXYT(T));
            drive.centerOfRotation = new double[] { 0.13, 0, 0 };
        }
        drive.poseTruth.orientation = LinAlg.matrixToQuat(T);
        drive.poseTruth.pos = new double[] { T[0][3], T[1][3], 0 };
    }

    static final Shape shape = new SphereShape(0.4);
    public Shape getShape()
    {
        return shape;
    }

    static Model4 model4 = new Model4(null, Color.blue, 0.5);
    public VisObject getVisObject()
    {
        return getVisObject(null);
    }

    public VisObject getVisObject(Color c)
    {
        VzLines.Style style;
        if (c != null)
            style = new VzLines.Style(c, 5);
        else
        //else if (success())
            style = new VzLines.Style(vcd, 5);
        //else
        //    style = new VzLines.Style(Color.red, 2);

        if (c != null) {
            return new VisLighting(false, new VzLines(new VisVertexData(trajectoryTruth),
                                                      VzLines.LINE_STRIP,
                                                      style),
                                   getPose(),
                                   new Model4(null, c, 0.5));
        } else {
            return new VisLighting(false, new VzLines(new VisVertexData(trajectoryTruth),
                                                      VzLines.LINE_STRIP,
                                                      style),
                                   getPose(),
                                   model4);
        }
    }

    public void read(StructureReader ins) throws IOException
    {
        // XXX
    }

    public void write(StructureWriter outs) throws IOException
    {
        // XXX
    }

    public void setRunning(boolean run)
    {
        synchronized (runLock) {
            this.running = run;
        }
    }
}
