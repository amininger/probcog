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
    private static final double ROBOT_RAD = Util.getConfig().requireDouble("robot.geometry.radius");
    private static final double BACK_THETA = 16*Math.PI/36;
    private static final double FRONT_THETA = 6*Math.PI/36;
    private static final double MAX_V = 0.5;
    private static final double MIN_V = 0.4;

    private PeriodicTasks tasks = new PeriodicTasks(1);
    private ExpiringMessageCache<laser_t> laserCache =
        new ExpiringMessageCache<laser_t>(.2);
    private ExpiringMessageCache<pose_t> poseCache =
        new ExpiringMessageCache<pose_t>(.2);

    Direction dir;
    private enum Direction { LEFT, RIGHT }
    private double goalDistance = 0.75;
    private double targetHeading = Double.MAX_VALUE;

    // Task State
    boolean oriented = false;
    int startIdx = -1;
    int finIdx = -1;

    // State for PID
    //double K_d = 0.05;
    double K_d = 0.001;
    double lastRange = -1;

    private class UpdateTask implements PeriodicTasks.Task
    {
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
    }

    // Publicly exposed initializer, used in monte-carlo sim
    public void init(laser_t laser)
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
    }

    // P controller to do wall avoidance. More logic could be added...
    private void update(laser_t laser, pose_t pose, double dt)
    {
        diff_drive_t dd;

        if (targetHeading == Double.MAX_VALUE)
            oriented = true;

        if (!oriented) {
            dd = orient(pose, targetHeading);
        } else {
            dd = drive(laser, dt);
        }

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
            dd.left = 0.7 * delta / Math.PI;
            dd.right = 0.7 * delta / -Math.PI;
        }

        // Friction sucks
        if (dd.left > 0) {
            dd.left = Math.max(dd.left, 0.1);
            dd.right = Math.min(dd.right, -0.1);
        } else {
            dd.left = Math.min(dd.left, -0.1);
            dd.right = Math.max(dd.right, 0.1);
        }

        return dd;
    }

    // Publicly exposed diff drive fn (can be used repeatedly as if static obj)
    // after initialized once
    public diff_drive_t drive(laser_t laser, double dt)
    {
        double[] xAxis = new double[] {1.0, 0};
        double[] yAxis = new double[] {0, 1.0};

        // Initialize no-speed diff drive
        diff_drive_t dd = new diff_drive_t();
        dd.utime = TimeUtil.utime();
        dd.left_enabled = dd.right_enabled = true;
        dd.left = 0;
        dd.right = 0;

        double r = Double.MAX_VALUE;
        double rFront = Double.MAX_VALUE;
        for (int i = startIdx; i <= finIdx; i++) {
            // Handle error states
            if (laser.ranges[i] < 0)
                continue;

            double t = laser.rad0 + laser.radstep*i;
            double w = MathUtil.clamp(Math.abs(Math.sin(t)),
                                      Math.sin(Math.PI/6),
                                      1.0);
            r = Math.min(r, w*laser.ranges[i]);
        }

        for (int i = 0; i < laser.nranges; i++) {
            // Handle error states
            if (laser.ranges[i] < 0)
                continue;

            double t = laser.rad0 + laser.radstep*i;

            // Points in front
            double[] xy = new double[] {laser.ranges[i]*Math.cos(t),
                                        laser.ranges[i]*Math.sin(t)};
            double width = Math.abs(LinAlg.dotProduct(xy, yAxis));
            if (width > 0.3)
                continue;

            double dist = LinAlg.dotProduct(xy, xAxis);
            if (dist > 0.1) {
                rFront = Math.min(rFront, dist);
            }
        }

        double deriv = 0;
        if (lastRange > 0)
            deriv = (r - lastRange)/dt; // [m/s] of change
        lastRange = r;

        // XXX
        double G_weight = Math.sqrt(goalDistance);
        double K_p = (1.0 - r/G_weight);
        double prop = MathUtil.clamp(0.5 + K_p, -1.0, 1.0);//0.65);

        //double nearSpeed = 0.5;
        //double farSpeed = MathUtil.clamp(prop + K_d*deriv, -1.0, 1.0);
        double farSpeed = 0.5;
        double nearSpeed = MathUtil.clamp(prop + K_d*deriv, 0, 1.0);    // XXX
        double max = Math.max(Math.abs(nearSpeed), Math.abs(farSpeed));
        if (max < 0.01) {
            nearSpeed = farSpeed = 0;
        } else {
            nearSpeed = MAX_V*nearSpeed/max;
            farSpeed = MAX_V*farSpeed/max;
        }

        // Special case turn-in-place when near a dead end in front
        //System.out.printf("%f\n", rFront);
        if (rFront < goalDistance + 0.5) {
            nearSpeed = MIN_V;
            farSpeed = -MIN_V;
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

    /** Strictly for use for parameter checking */
    public FollowWall()
    {
    }

    public FollowWall(Map<String, TypedValue> parameters)
    {
        //System.out.println("FOLLOW WALL");

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

    // Ignore heading for now
    public int hashCode()
    {
        return dir.hashCode() ^ new Double(goalDistance).hashCode();
    }

    public boolean equals(Object o)
    {
        if (o == null)
            return false;
        else if (!(o instanceof FollowWall))
            return false;
        FollowWall fw = (FollowWall)o;
        return dir == fw.dir && goalDistance == fw.goalDistance;
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

    public String getSide()
    {
        if (dir == Direction.LEFT)
            return "LEFT";
        return "RIGHT";
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
                                      options,
                                      true));

        params.add(new TypedParameter("distance",
                                      TypedValue.TYPE_DOUBLE,
                                      false));  // Could have range limits...

        params.add(new TypedParameter("heading",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(-Math.PI),
                                      new TypedValue(Math.PI),
                                      false));

        return params;
    }
}
