package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.commands.*;
import probcog.lcmtypes.*;

/** Given a heading (yaw) to follow relative to the global coordinate frame, try
 *  to drive the robot in that direction, avoiding obstacles along the way.
 *  Note: Will get stuck in local minima.
 **/
public class FollowHeading implements ControlLaw, LCMSubscriber
{
    private static final double FH_HZ = 100;
    private static final double MIN_THETA = -Math.PI/2;
    private static final double MAX_THETA = Math.PI/2;
    private static final double HEADING_THRESH = Math.toRadians(5.0);
    private static final double K_d = 0.05;

    double targetHeading = 0;
    double distance = 0.75;   // XXX distance to be away from wall

    PeriodicTasks tasks = new PeriodicTasks(1);
    ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);
    ExpiringMessageCache<laser_t> laserCache = new ExpiringMessageCache<laser_t>(0.2);

    private class UpdateTask implements PeriodicTasks.Task
    {
        diff_drive_t lastDD;
        double rRightLast = Double.MAX_VALUE;
        double rLeftLast = Double.MAX_VALUE;

        boolean oriented = false;
        int startIdx = -1;
        int midIdx = -1;
        int finIdx = -1;

        public UpdateTask()
        {
            lastDD = new diff_drive_t();
            lastDD.right = 0;
            lastDD.left = 0;
        }

        public void run(double dt)
        {
            laser_t laser = laserCache.get();
            if (laser == null)
                return;

            pose_t pose = poseCache.get();
            if (pose == null)
                return;

            // Initialization step
            if (startIdx < 0)
                init(laser);

            // Turn in place until facing the correct direction. Then drive
            // in the general direction of the heading, following walls as
            // necessary. If forced to turn more than 90 degrees off from
            // ideal heading once underway, halt.
            update(laser, pose, dt);
        }

        private void init(laser_t laser)
        {
            startIdx = MathUtil.clamp((int)((MIN_THETA-laser.rad0)/laser.radstep), 0, laser.nranges-1);
            midIdx = MathUtil.clamp((int)((0-laser.rad0)/laser.radstep), 0, laser.nranges-1);
            finIdx = MathUtil.clamp((int)((MAX_THETA-laser.rad0)/laser.radstep), 0, laser.nranges-1);
        }

        private void update(laser_t laser, pose_t pose, double dt)
        {
            // Initialize no-speed diff drive
            diff_drive_t dd = new diff_drive_t();

            if (!oriented)
                dd = orient(pose, targetHeading);
            else
                dd = drive(laser, pose, targetHeading, dt);

            LCM.getSingleton().publish("DIFF_DRIVE", dd);
        }

        private diff_drive_t orient(pose_t pose, double heading)
        {
            double[] rpy = LinAlg.quatToRollPitchYaw(pose.orientation);

            diff_drive_t dd = new diff_drive_t();
            dd.utime = TimeUtil.utime();
            dd.left_enabled = dd.right_enabled = true;
            dd.left = 0;
            dd.right = 0;

            double delta = MathUtil.mod2pi(rpy[2] - heading);

            if (Math.abs(delta) < HEADING_THRESH) {
                oriented = true;
            } else {
                dd.left = 0.7 * delta / (Math.PI/2);
                dd.right = 0.7 * delta / (-Math.PI/2);
            }

            return dd;
        }

        private diff_drive_t drive(laser_t laser, pose_t pose, double heading, double dt)
        {
            double[] rpy = LinAlg.quatToRollPitchYaw(pose.orientation);

            diff_drive_t dd = new diff_drive_t();
            dd.utime = TimeUtil.utime();
            dd.left_enabled = dd.right_enabled = true;
            dd.left = 0;
            dd.right = 0;

            // Try to drive towards heading, first.
            double delta = MathUtil.mod2pi(rpy[2] - heading);
            dd.left = 0.5 + .3*(delta/(Math.PI/2));
            dd.right = 0.5 + .3*(delta/(-Math.PI/2));

            // Avoid anything that gets in the way, within reason
            double max = Math.max(dd.left, dd.right);
            double rLeft = Double.MAX_VALUE;
            double rRight = Double.MAX_VALUE;
            ArrayList<Double> rightSamples = new ArrayList<Double>();
            ArrayList<Double> leftSamples = new ArrayList<Double>();
            for (int i = startIdx; i < finIdx; i++) {
                double t = laser.rad0+laser.radstep*i;
                double w = Math.max(Math.abs(Math.cos(t)), Math.abs(Math.sin(t)));
                if (i < midIdx) {
                    rRight = Math.min(w*laser.ranges[i], rRight);
                    rightSamples.add(w*laser.ranges[i]);
                } else {
                    rLeft = Math.min(w*laser.ranges[i], rLeft);
                    leftSamples.add(w*laser.ranges[i]);
                }
            }

            // This causes walls to "push" on the robot. Repulsor fields!
            double leftPush = Math.pow(distance/Math.max(distance, rRight), 3);
            double rightPush = Math.pow(distance/Math.max(distance, rLeft), 3);
            double rLeftDeriv = (rLeft - rLeftLast)/dt; // [m/s]
            double rRightDeriv = (rRight - rRightLast)/dt;  // [m/s]
            dd.left = dd.left - (dd.left*leftPush) - (K_d*rLeftDeriv);
            dd.right = dd.right - (dd.right*rightPush) - (K_d*rRightDeriv);

            // Normalize
            max = Math.max(Math.abs(dd.left), Math.abs(dd.right));
            if (max > K_d) {
                dd.left /= max;
                dd.right /= max;
            }

            rRightLast = rRight;
            rLeftLast = rLeft;

            return dd;
        }
    }

    /** Strictly for use for parameter checking */
    public FollowHeading()
    {
    }

    public FollowHeading(Map<String, TypedValue> parameters)
    {
        System.out.println("FOLLOW HEADING");

        assert (parameters.containsKey("heading"));
        targetHeading = parameters.get("heading").getDouble();

        if (parameters.containsKey("distance")) {
            distance = Math.abs(parameters.get("distance").getDouble());
        }

        LCM.getSingleton().subscribe("POSE", this);
        LCM.getSingleton().subscribe("LASER", this);
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
        if ("LASER".equals(channel)) {
            laser_t laser = new laser_t(ins);
            laserCache.put(laser, laser.utime);
        } else if ("POSE".equals(channel)) {
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
        params.add(new TypedParameter("heading",    // [rad] in global coords
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(-Math.PI),
                                      new TypedValue(Math.PI)));
        params.add(new TypedParameter("distance",   // [m] from wall
                                      TypedValue.TYPE_DOUBLE));
        return params;
    }

}
