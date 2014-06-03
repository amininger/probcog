package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.commands.*;
import probcog.util.*;

public class FollowWall implements ControlLaw, LCMSubscriber
{
    private static final double FW_HZ = 100;
    private static final double HEADING_THRESH = Math.toRadians(5.0);
    private static final double ROBOT_RAD = Util.getDomainConfig().requireDouble("robot.geometry.radius");
    private static final double BACK_THETA = Math.PI/2;
    private static final double FRONT_THETA = -Math.PI/4;
    private static final double MAX_V = 1.0;

    private PeriodicTasks tasks = new PeriodicTasks(1);
    private ExpiringMessageCache<laser_t> laserCache =
        new ExpiringMessageCache<laser_t>(.2);
    private ExpiringMessageCache<pose_t> poseCache =
        new ExpiringMessageCache<pose_t>(.2);

    Direction dir;
    private enum Direction { LEFT, RIGHT }
    private double goalDistance = Double.MAX_VALUE;
    private double targetHeading = 0;

    private class UpdateTask implements PeriodicTasks.Task
    {
        boolean oriented = false;
        int startIdx = -1;
        int finIdx = -1;

        // State for PID
        double K_d = 0.3;
        double lastRange = -1;

        public void run(double dt)
        {
            laser_t laser = laserCache.get();
            if (laser == null)
                return;

            pose_t pose = poseCache.get();
            if (pose == null)
                return;

            // Initialization step.
            // Find the laser beam we will observe for distance. If we are using
            // a 180 or greater degree sensor, this should be 90 degrees CW or
            // 90 degress CCW from the front of the robot, in all likelihood.
            // Otherwise, we choose the furthest beam out to that side and correct
            // our range measurements accordingly.
            if (startIdx < 0)
                init(laser);

            // Update the movement of the robot. Basically, swing towards wall
            // when range > desired, else swing back the other way. Some extra
            // logic to prevent running into walls.
            update(laser, pose, dt);
        }

        private void init(laser_t laser)
        {
            // Find indices at which to start/stop scanning
            switch (dir) {
                case LEFT:
                    finIdx = MathUtil.clamp((int)((BACK_THETA - laser.rad0)/laser.radstep), 0, laser.nranges-1);
                    startIdx = MathUtil.clamp((int)((FRONT_THETA - laser.rad0)/laser.radstep), 0, laser.nranges-1);
                    break;
                case RIGHT:
                    startIdx = MathUtil.clamp((int)((-BACK_THETA - laser.rad0)/laser.radstep), 0, laser.nranges-1);
                    finIdx = MathUtil.clamp((int)((-FRONT_THETA - laser.rad0)/laser.radstep), 0, laser.nranges-1);
                    break;
                default:
                    assert (false);
                    break;
            }

            assert (startIdx <= finIdx);

            // If no goal distance was specified, find one
            if (goalDistance == Double.MAX_VALUE) {
                for (int i = startIdx; i <= finIdx; i++) {
                    goalDistance = Math.min(goalDistance, laser.ranges[i]);
                }
            }
        }

        // P controller to do wall avoidance. More logic could be added...
        private void update(laser_t laser, pose_t pose, double dt)
        {
            diff_drive_t dd;

            if (!oriented) {
                dd = orient(pose, targetHeading);
            } else {
                dd = drive(laser, dt);
            }

            LCM.getSingleton().publish("DIFF_DRIVE", dd);
        }

        private diff_drive_t drive(laser_t laser, double dt)
        {
            // Initialize no-speed diff drive
            diff_drive_t dd = new diff_drive_t();
            dd.utime = TimeUtil.utime();
            dd.left_enabled = dd.right_enabled = true;
            dd.left = 0;
            dd.right = 0;

            double r = Double.MAX_VALUE;
            for (int i = startIdx; i <= finIdx; i++) {
                //double w = 1.0;
                //if (finIdx - i > (finIdx - startIdx)/2)
                //    w = .75;
                double w = MathUtil.clamp(Math.abs(Math.sin(laser.rad0+laser.radstep*i)),
                                          Math.sin(Math.PI/6), 1.0);
                r = Math.min(r, w*laser.ranges[i]);
            }
            double deriv = 0;
            if (lastRange > 0)
                deriv = (r - lastRange)/dt; // [m/s] of change
            lastRange = r;

            // XXX
            double prop = MathUtil.clamp(-0.5 + r/goalDistance, -1.0, 1.0);

            double nearSpeed = 0.5;
            double farSpeed = MathUtil.clamp(prop + K_d*deriv, -1.0, 1.0);
            double max = Math.max(Math.abs(nearSpeed), Math.abs(farSpeed));
            if (max < 0.01) {
                nearSpeed = farSpeed = 0;
            } else {
                nearSpeed = MAX_V*nearSpeed/max;
                farSpeed = MAX_V*farSpeed/max;
            }

            switch (dir) {
                case LEFT:
                    dd.left = nearSpeed;
                    dd.right = farSpeed;
                    break;
                case RIGHT:
                    dd.right = nearSpeed;
                    dd.left = farSpeed;
                    break;
            }

            return dd;
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
                dd.left = 0.7 * delta / Math.PI;
                dd.right = 0.7 * delta / -Math.PI;
            }

            return dd;
        }
    }

    /** Strictly for use for parameter checking */
    public FollowWall()
    {
    }

    public FollowWall(Map<String, TypedValue> parameters)
    {
        System.out.println("FOLLOW WALL");

        assert (parameters.containsKey("side"));
        if (parameters.get("side").getByte() < 0)
            dir = Direction.RIGHT;
        else
            dir = Direction.LEFT;

        if (parameters.containsKey("distance"))
            goalDistance = Math.abs(parameters.get("distance").getDouble());

        if (parameters.containsKey("heading"))
            targetHeading = parameters.get("heading").getDouble();

        LCM.getSingleton().subscribe("LASER", this);
        LCM.getSingleton().subscribe("POSE", this);
        tasks.addFixedDelay(new UpdateTask(), 1.0/FW_HZ);
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
        return "FOLLOW_WALL";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        ArrayList<TypedValue> options = new ArrayList<TypedValue>();
        options.add(new TypedValue((byte)-1));
        options.add(new TypedValue((byte)1));
        params.add(new TypedParameter("side",
                                      TypedValue.TYPE_BYTE,
                                      options));

        params.add(new TypedParameter("distance",
                                      TypedValue.TYPE_DOUBLE));  // Could have range limits...

        params.add(new TypedParameter("heading",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(-Math.PI),
                                      new TypedValue(Math.PI)));

        return params;
    }
}