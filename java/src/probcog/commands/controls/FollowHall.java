package probcog.commands.controls;

// XXX Test
import java.awt.*;
import javax.swing.*;
import april.vis.*;
import april.jmat.geom.*;
// ----------------

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

import probcog.commands.*;
import probcog.util.*;

import probcog.lcmtypes.*;
import magic2.lcmtypes.*;

// TODO: Add orientation stuff if needed
public class FollowHall implements ControlLaw, LCMSubscriber
{
    // General parameters
    private static final int FH_HZ = 100;

    // Hall parameters
    private static final double MIN_RANGE = 3.0;
    private static final double THETA_SWEEP = Math.atan2(1.0, MIN_RANGE);
    private static final double MIN_THETA = -Math.PI/2 - THETA_SWEEP;
    private static final double MAX_THETA = Math.PI/2 + THETA_SWEEP;
    private static final int ORIENTATION_SAMPLES = FH_HZ*5;

    // Range parameters
    private static final double WALL_WEIGHT = 0.20;
    private static final double MIN_R_THETA = -Math.PI/2;
    private static final double MAX_R_THETA = Math.PI/2;
    private static final double WALL_RIGHT_CEIL= 3.0;
    private static final double WALL_LEFT_CEIL = 3.0;

    LCM lcm = LCM.getSingleton();
    String laserChannel = Util.getConfig().getString("robot.lcm.laser_channel", "LASER");
    String poseChannel = Util.getConfig().getString("robot.lcm.pose_channel", "POSE");
    String driveChannel = Util.getConfig().getString("robot.lcm.drive_channel", "DIFF_DRIVE");


    PeriodicTasks tasks = new PeriodicTasks(1);
    ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);
    ExpiringMessageCache<laser_t> laserCache = new ExpiringMessageCache<laser_t>(0.2);

    int startIdx = -1;
    int midIdx = -1;
    int finIdx = -1;
    double meanOrientation = Double.MAX_VALUE;

    class UpdateTask implements PeriodicTasks.Task
    {
        // XXX Debugging
        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;

        public UpdateTask()
        {
        }

        public void run(double dt)
        {
            laser_t laser = laserCache.get();
            if (laser == null)
                return;

            pose_t pose = poseCache.get();
            if (pose == null)
                return;

            DriveParams params = new DriveParams();
            params.laser = laser;
            params.pose = pose;
            params.dt = dt;
            diff_drive_t dd = drive(params);
            lcm.publish(driveChannel, dd);
        }

    }

    private void init(laser_t laser, pose_t pose)
    {
        meanOrientation = LinAlg.quatToRollPitchYaw(pose.orientation)[2];

        startIdx = MathUtil.clamp((int)((MIN_R_THETA-laser.rad0)/laser.radstep), 0, laser.nranges-1);
        midIdx = MathUtil.clamp((int)((0-laser.rad0)/laser.radstep), 0, laser.nranges-1);
        finIdx = MathUtil.clamp((int)((MAX_R_THETA-laser.rad0)/laser.radstep), 0, laser.nranges-1);
    }

    //private diff_drive_t drive(laser_t laser, pose_t pose)
    public diff_drive_t drive(DriveParams params)
    {
        laser_t laser = params.laser;
        pose_t pose = params.pose;
        double dt = params.dt;

        if (meanOrientation == Double.MAX_VALUE)
            init(laser, pose);

        double currentOrientation = LinAlg.quatToRollPitchYaw(pose.orientation)[2];
        double w0 = (ORIENTATION_SAMPLES-1.0)/ORIENTATION_SAMPLES;
        double w1 = 1.0 - w0;
        meanOrientation = w0*meanOrientation + w1*MathUtil.mod2pi(meanOrientation, currentOrientation);
        meanOrientation = MathUtil.mod2pi(meanOrientation);

        // Find left and right ranges. This should help push us into the hall
        // center
        double rLeft = Double.MAX_VALUE;
        double rRight = Double.MAX_VALUE;
        for (int i = startIdx; i < finIdx; i++) {
            // Handle error state
            if (laser.ranges[i] < 0)
                continue;

            // XXX This is a magical hand-calibration
            if (laser.ranges[i] > 6.0)
                continue;
            double t = laser.rad0+laser.radstep*i;
            double w = Math.max(Math.abs(Math.sin(t)), 0.1); // XXX as is this
            if (i < midIdx) {
                rRight = Math.min(w*laser.ranges[i], rRight);
            } else {
                rLeft = Math.min(w*laser.ranges[i], rLeft);
            }
        }

        // Locate the direction closest to being in front of us (without
        // being behind us) that is free.
        // (Later, we'll add some slight push back by walls to try to recenter)
        int SAMPLES = (int)Math.ceil(THETA_SWEEP*2/laser.radstep);
        LinkedList<Double> slidingRanges = new LinkedList<Double>();
        int unsafe = 0;
        double bestTheta = Double.MAX_VALUE;
        double bestDiff = Double.MAX_VALUE;
        for (int i = 0; i < laser.nranges; i++) {
            // Handle error states
            if (laser.ranges[i] < 0)
                continue;

            double t = laser.rad0 + i*laser.radstep;
            if (t < MIN_THETA || t > MAX_THETA)
                continue;

            // Measure how many points in our current window are
            // unsafe. If there are any, this prevents us from
            // forcing the robot in this direction
            if (laser.ranges[i] < MIN_RANGE)
                unsafe++;

            slidingRanges.addLast((double)laser.ranges[i]);
            if (slidingRanges.size() > SAMPLES) {
                double r = slidingRanges.removeFirst();
                if (r < MIN_RANGE)
                    unsafe--;
            }
            if (slidingRanges.size() != SAMPLES)
                continue;

            // Map middle theta to global orientation
            double middleTheta = laser.rad0 + (i-SAMPLES/2)*laser.radstep;
            double actualTheta = MathUtil.mod2pi(middleTheta + currentOrientation);
            double diff = Math.abs(MathUtil.mod2pi(meanOrientation, actualTheta) - meanOrientation);
            if (unsafe == 0 && diff < bestDiff) {
                bestDiff = diff;
                bestTheta = middleTheta;
            }
        }

        // Drive in that direction
        diff_drive_t dd = new diff_drive_t();
        dd.utime = TimeUtil.utime();
        dd.left_enabled = dd.right_enabled = true;
        dd.left = dd.right = 0;

        if (bestTheta != Double.MAX_VALUE) {
            if (bestTheta > 0) {
                dd.right = 1.0;
                dd.left = Math.cos(bestTheta);
            } else {
                dd.left = 1.0;
                dd.right = Math.cos(bestTheta);
            }
        } else {
            return dd;
        }

        // Add in bias to drive down center (or just right of center) of hall
        double rightPush = (WALL_RIGHT_CEIL - Math.min(WALL_RIGHT_CEIL, rRight))/WALL_RIGHT_CEIL;
        double leftPush = (WALL_LEFT_CEIL - Math.min(WALL_LEFT_CEIL, rLeft))/WALL_LEFT_CEIL;
        dd.right += WALL_WEIGHT*rightPush;
        dd.left += WALL_WEIGHT*leftPush;

        // Normalize
        double max = Math.max(Math.abs(dd.right), Math.abs(dd.left));
        assert (max != 0);
        dd.right /= max;
        dd.left /= max;

        return dd;
    }

    /** Strictly for use in parameter checking */
    public FollowHall()
    {
    }

    public FollowHall(Map<String, TypedValue> parameters)
    {
        // Parameters
        // XXX Are there any?

        tasks.addFixedDelay(new UpdateTask(), 1.0/FH_HZ);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.err.println("WRN: Error receving message from channel " + channel + ": "+ex);
        }
    }

    synchronized void messageReceivedEx(LCM lcm, String channel,
            LCMDataInputStream ins) throws IOException
    {
        if (laserChannel.equals(channel)) {
            laser_t laser = new laser_t(ins);
            laserCache.put(laser, laser.utime);
        } else if (poseChannel.equals(channel)) {
            pose_t pose = new pose_t(ins);
            poseCache.put(pose, pose.utime);
        }
    }

    /** Start/stop the execution of the control law.
     *
     *  @param run  True causes the control law to begin execution, false stops it
     **/
    public void setRunning(boolean run)
    {
        if (run) {
            lcm.subscribe(laserChannel, this);
            lcm.subscribe(poseChannel, this);
        } else {
            lcm.unsubscribe(laserChannel, this);
            lcm.unsubscribe(poseChannel, this);
        }
        tasks.setRunning(run);
    }

    /** Get the name of this control law. Mostly useful for debugging purposes.
     *
     *  @return The name of the control law
     **/
    public String getName()
    {
        return "FOLLOW_HEADING";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();

        return params;
    }

    // ======================================================================
    // === TEST                                                           ===
    // ======================================================================

    class TestThread extends Thread
    {
        int fps = 60;
        VisWorld vw;

        double meanOrientation = Double.MAX_VALUE;

        public TestThread(VisWorld vw)
        {
            this.vw = vw;
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/fps);
                laser_t laser = laserCache.get();
                if (laser == null)
                    continue;

                pose_t pose = poseCache.get();
                if (pose == null)
                    continue;

                init(pose);

                process(laser, pose);
            }
        }

        private void init(pose_t pose)
        {
            if (meanOrientation == Double.MAX_VALUE)
                meanOrientation = LinAlg.quatToRollPitchYaw(pose.orientation)[2];
        }

        private void process(laser_t laser, pose_t pose)
        {
            double currentOrientation = LinAlg.quatToRollPitchYaw(pose.orientation)[2];
            double w0 = (ORIENTATION_SAMPLES-1.0)/ORIENTATION_SAMPLES;
            double w1 = 1.0 - w0;
            meanOrientation = w0*meanOrientation + w1*MathUtil.mod2pi(meanOrientation, currentOrientation);
            meanOrientation = MathUtil.mod2pi(meanOrientation);

            ArrayList<double[]> points = new ArrayList<double[]>();
            for (int i = 0; i < laser.nranges; i++) {
                double t = laser.rad0 + i*laser.radstep;
                double r = laser.ranges[i];
                points.add(new double[] {r*Math.cos(t), r*Math.sin(t)});
            }

            // Render points
            VisWorld.Buffer vb = vw.getBuffer("laser");
            vb.addBack(new VzPoints(new VisVertexData(points),
                                    new VzPoints.Style(Color.yellow, 2)));
            vb.swap();

            // Weight direction based on where we've been facing

            // Locate the direction closest to being in front of us (without
            // being behind us) that is free.
            // (Later, we'll add some slight push back by walls to try to recenter)
            int SAMPLES = (int)Math.ceil(THETA_SWEEP*2/laser.radstep);
            LinkedList<Double> slidingRanges = new LinkedList<Double>();
            int unsafe = 0;
            double bestTheta = Double.MAX_VALUE;
            double bestDiff = Double.MAX_VALUE;
            for (int i = 0; i < laser.nranges; i++) {
                double t = laser.rad0 + i*laser.radstep;
                if (t < MIN_THETA || t > MAX_THETA)
                    continue;
                if (laser.ranges[i] < MIN_RANGE)
                    unsafe++;
                slidingRanges.addLast((double)laser.ranges[i]);
                if (slidingRanges.size() > SAMPLES) {
                    double r = slidingRanges.removeFirst();
                    if (r < MIN_RANGE)
                        unsafe--;
                }
                if (slidingRanges.size() != SAMPLES)
                    continue;

                double middleTheta = laser.rad0 + (i-SAMPLES/2)*laser.radstep;
                // Map middle theta to global orientation
                double actualTheta = MathUtil.mod2pi(middleTheta + currentOrientation);
                double diff = Math.abs(MathUtil.mod2pi(meanOrientation, actualTheta) - meanOrientation);
                if (unsafe == 0 && diff < bestDiff) {
                    bestDiff = diff;
                    bestTheta = middleTheta;
                }
            }

            double radius = MIN_RANGE;
            if (bestTheta == Double.MAX_VALUE) {
                radius = 0;
                bestTheta = 0;
            }
            vb = vw.getBuffer("direction");
            VisVertexData vvd = new VisVertexData(new double[2], new double[] {radius*Math.cos(bestTheta), radius*Math.sin(bestTheta)});
            vb.addBack(new VzLines(vvd, VzLines.LINES, new VzLines.Style(Color.red, 2)));
            vb.swap();
        }
    }

    public void launchTest(VisWorld vw)
    {
        (new TestThread(vw)).start();
    }

    // Test functionality
    public static void main(String[] args)
    {
        JFrame jf = new JFrame("Hall following test");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 800);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        VzGrid.addGrid(vw);
        jf.add(vc, BorderLayout.CENTER);

        jf.setVisible(true);

        (new FollowHall()).launchTest(vw);
    }
}
