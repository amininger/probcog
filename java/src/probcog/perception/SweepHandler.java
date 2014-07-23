package probcog.perception;

import java.util.*;
import java.awt.*;
import java.awt.image.*;
import javax.swing.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.util.*;
import april.lcmtypes.*;
import april.vis.*;

import probcog.sensor.*;
import probcog.lcmtypes.*;
import probcog.util.*;

public class SweepHandler implements DynamixelPoseLidar.Listener
{
    static final boolean GUI = false;

    LCM lcm = LCM.getSingleton();

    PoseTracker poseTracker = PoseTracker.getSingleton();

    DynamixelPoseLidar lidar;
    Object queueLock = new Object();
    ArrayList<DynamixelPoseLidar.Data> sweepQueue = null;
    int min_slices = 5;    // Discard sweeps with fewer than this many slices

    // GridMap constants
    static final double MPP = 0.05;      // [meters/px]
    static final double SIZE_X = 20.0;  // [meters]
    static final double SIZE_Y = 20.0;  // [meters]
    double minHeight = Util.getConfig().requireDouble("obstacle.min_height");
    double maxHeight = Util.getConfig().requireDouble("obstacle.max_height");
    Object mapLock = new Object();
    GridMap map = null;

    /** Broadcasts fake "LASER" messages for control laws to operate on. These
     *  messages are laser_t still, but take the 3D information coming in and
     *  make a conservative 2D scan based on the closest obstacles.
     *
     *  We cheat a LOT here by assuming our ground is flat and the robot is level.
     */
    private class LaserThread extends Thread
    {
        VisWorld vw;
        int hz = 20;

        public LaserThread(VisWorld vw)
        {
            this.vw = vw;
        }

        public void run()
        {
            laser_t laser = new laser_t();
            laser.rad0 = (float)Math.toRadians(-135);
            laser.radstep = (float)Math.toRadians(0.5);
            laser.nranges = (int)(2*Math.abs(laser.rad0/laser.radstep));
            laser.ranges = new float[laser.nranges];

            Tic tic = new Tic();
            while (true)
            {
                double time = tic.toctic();
                int delay = (int)Math.max(0, 1000/hz - 1000*time);
                //System.out.println("Pause: "+delay);
                TimeUtil.sleep(delay);

                GridMap gm = null;
                synchronized (mapLock) {
                    gm = map;
                }
                if (gm == null)
                    continue;

                laser.utime = TimeUtil.utime();

                // This has logging flaws XXX
                pose_t pose = poseTracker.get(TimeUtil.utime());
                if (pose == null)
                    continue;

                double[] xyt = LinAlg.quatPosToXYT(pose.orientation,
                                                   pose.pos);

                // XXX Not necessarily true :/
                //assert (xyt[0] >= gm.x0 && xyt[0] < gm.x0+gm.width*gm.metersPerPixel &&
                //        xyt[1] >= gm.y0 && xyt[1] < gm.y0+gm.height*gm.metersPerPixel);

                // Expensive grid map processing
                float step = (float)gm.metersPerPixel/2;
                for (int i = 0; i < laser.nranges; i++) {
                    laser.ranges[i] = -0.00001f;
                    float theta = (float)(xyt[2]+laser.rad0+laser.radstep*i);
                    double x = 0, y = 0;
                    for (float r = 0.0f; r < 5.0f; r+=step) {
                        x = xyt[0] + r*Math.cos(theta);
                        y = xyt[1] + r*Math.sin(theta);

                        if (gm.getValue(x, y) != 0) {
                            laser.ranges[i] = r;
                            break;
                        }
                    }
                }
                lcm.publish("LASER", laser);
            }
        }
    }

    private class WorkerThread extends Thread
    {
        VisWorld vw;

        public WorkerThread(VisWorld vw)
        {
            this.vw = vw;
        }

        public void run()
        {
            while (true) {
                ArrayList<DynamixelPoseLidar.Data> datas;

                synchronized (queueLock) {
                    datas = sweepQueue;
                    sweepQueue = null;

                    if (datas == null) {
                        try {
                            queueLock.wait();
                        } catch (InterruptedException ex) {}
                    }
                }

                if (datas != null) {
                    try {
                        processDatas(datas);

                    } catch (Exception ex) {
                        ex.printStackTrace();
                        System.exit(-1);
                    }
                }
            }
        }

        private void processDatas(ArrayList<DynamixelPoseLidar.Data> datas)
        {
            // Drop incomplete sweeps
            if (datas.size() < min_slices)
                return;

            // Initialize map position based on current pose
            pose_t pose = poseTracker.get(TimeUtil.utime());
            if (pose == null)
                return;
            GridMap gm = GridMap.makeMeters(pose.pos[0]-SIZE_X/2, pose.pos[1]-SIZE_Y/2, SIZE_X, SIZE_Y, MPP, 0);

            // For each sweep, add those points to the map when relevant. Anything
            // between min and max height gets added
            for (DynamixelPoseLidar.Data data: datas) {
                // Project the points

                // body to local frame
                double[][] B2L = LinAlg.quatPosToMatrix(data.pose.orientation, data.pose.pos);

                // sensor to body
                double[][] S2B = LinAlg.multiplyMany(ConfigUtil.getRigidBodyTransform(Util.getConfig(), "HOKUYO_LIDAR"),
                                                     LinAlg.rotateY(-data.status.position_radians),
                                                     LinAlg.translate(0,0,Util.getConfig().requireDouble("HOKUYO_LIDAR.zoffset")));

                ArrayList<double[]> pointsRaw = new ArrayList<double[]>();
                for (int i = 0; i < data.laser.nranges; i++) {
                    double r = data.laser.ranges[i];

                    // Skip error codes
                    if (r < 0)
                        continue;

                    double t = data.laser.rad0 + data.laser.radstep*i;

                    double[] xyz = new double[3];
                    xyz[0] = r*Math.cos(t);
                    xyz[1] = r*Math.sin(t);
                    pointsRaw.add(xyz);

                    // TODO: Glance points filter

                    // Sensor to local frame
                    double[][] S2L = LinAlg.multiplyMany(B2L, S2B);
                    ArrayList<double[]> pointsLocal = LinAlg.transform(S2L, pointsRaw);

                    // Put the points in the map
                    for (double[] p: pointsLocal) {
                        // Skip non-obstacle points
                        if (p[2] < minHeight || p[2] > maxHeight)
                            continue;
                        gm.setValue(p[0], p[1], (byte)255);
                    }

                    if (GUI) {
                        VisWorld.Buffer vb = vw.getBuffer("3dpoints");
                        vb.addBack(new VzPoints(new VisVertexData(pointsLocal),
                                                new VzPoints.Style(Color.red, 2)));
                        vb.swap();
                    }

                }
            }
            synchronized (mapLock) {
                map = gm;
            }
        }
    }

    public SweepHandler()
    {
        lidar = new DynamixelPoseLidar(5);
        lidar.addListener(this);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VzGrid.addGrid(vw);

        (new WorkerThread(vw)).start();
        (new LaserThread(vw)).start();

        // GUI Setup
        if (GUI) {
            JFrame jf = new JFrame("Sweep Handler");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setLayout(new BorderLayout());
            jf.setSize(800, 600);

            VisCanvas vc = new VisCanvas(vl);
            jf.add(vc, BorderLayout.CENTER);

            jf.setVisible(true);
        }
    }

    // Unused
    public void handleData(DynamixelPoseLidar.Data d)
    {
    }

    /** Handle incoming sweeps from the DynamixelPoseLidar. */
    public void handleSweep(ArrayList<DynamixelPoseLidar.Data> ds)
    {
        synchronized (queueLock) {
            if (sweepQueue != null)
                System.out.println("Dropping sweep @ "+TimeUtil.utime());

            sweepQueue = ds;
            queueLock.notifyAll();
        }
    }

    static public void main(String[] args)
    {
        new SweepHandler();
    }
}
