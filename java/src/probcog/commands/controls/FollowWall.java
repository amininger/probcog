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
    private static final double ROBOT_RAD = Util.getDomainConfig().requireDouble("robot.geometry.radius");
    private PeriodicTasks tasks = new PeriodicTasks(1);
    private ExpiringMessageCache<laser_t> laserCache =
        new ExpiringMessageCache<laser_t>(.2);

    Direction dir;
    private enum Direction { LEFT, RIGHT }
    private double goalDistance = -1;

    private class UpdateTask implements PeriodicTasks.Task
    {
        int idx = -1;       // laser_t beam index
        double offset;      // offset from 90 degrees if necessary

        double MAX_DIST = 5.0;  // [m]

        public void run(double dt)
        {
            laser_t laser = laserCache.get();
            if (laser == null)
                return;

            // Initialization step.
            // Find the laser beam we will observe for distance. If we are using
            // a 180 or greater degree sensor, this should be 90 degrees CW or
            // 90 degress CCW from the front of the robot, in all likelihood.
            // Otherwise, we choose the furthest beam out to that side and correct
            // our range measurements accordingly.
            if (idx < 0)
                init(laser);

            // Update the movement of the robot. Basically, swing towards wall
            // when range > desired, else swing back the other way. Some extra
            // logic to prevent running into walls.
            update(laser, dt);
        }

        private void init(laser_t laser)
        {
            // Find best laser range measurement
            double bestDistance = Double.POSITIVE_INFINITY;
            double dist = Double.POSITIVE_INFINITY;
            for (int i = 0; i < laser.nranges; i++) {
                double t = laser.rad0 + i*laser.radstep;
                switch (dir) {
                    case LEFT:
                        dist = Math.abs(Math.PI/2 - t);
                        break;
                    case RIGHT:
                        dist = Math.abs(-Math.PI/2 - t);
                        break;
                }
                if (dist < bestDistance) {
                    bestDistance = dist;
                    idx = i;
                    offset = dist;
                }
            }

            // If no distance was specified, try to maintain current distance
            if (goalDistance < 0) {
                goalDistance = Math.cos(offset)*laser.ranges[idx];
            }
        }

        // P controller to do wall avoidance. More logic could be added...
        private void update(laser_t laser, double dt)
        {
            assert (idx >= 0);
            double r = laser.ranges[idx];

            // Initialize no-speed diff drive
            diff_drive_t dd = new diff_drive_t();
            dd.utime = TimeUtil.utime();
            dd.left_enabled = dd.right_enabled = true;
            dd.left = 0;
            dd.right = 0;

            // Distance delta. Positive == too far, negative == too close
            double delta = MathUtil.clamp(Math.cos(offset)*r - goalDistance,
                                          -MAX_DIST,
                                          MAX_DIST);

            double nearSpeed = 0.5 - 0.5*(delta/MAX_DIST);
            double farSpeed = 0.5 + 0.5*(delta/MAX_DIST);

            // Correct for forward obstacles. If we get within a robot radius
            // of an obstacle for any of the beams in a 90 degree fan in front
            // of us, turn hard in the OPPOSITE direction of the wall we are
            // following.
            for (int i = 0; i < laser.nranges; i++) {
                double t = laser.rad0 + i*laser.radstep;
                if (-Math.PI/4 > t || Math.PI/4 < t)
                    continue;
                double dist = Math.cos(t)*laser.ranges[i];
                if (dist < ROBOT_RAD+.25) {
                    nearSpeed = 0.5;
                    farSpeed = -0.5;
                    break;
                }
            }

            double max = Math.max(nearSpeed, farSpeed);
            nearSpeed = nearSpeed/max;
            farSpeed = farSpeed/max;

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

            LCM.getSingleton().publish("DIFF_DRIVE", dd);
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

        LCM.getSingleton().subscribe("LASER", this);
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

        return params;
    }
}
