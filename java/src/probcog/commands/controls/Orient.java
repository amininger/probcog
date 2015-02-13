package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.lcmtypes.*;    // XXX
import april.util.*;
import april.jmat.*;

import probcog.commands.*;
import probcog.lcmtypes.*;  // XXX

/** Orient the robot to face along a particular heading. Automatically turns
 *  the most efficient direction.
 */
public class Orient implements ControlLaw, LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    static final double MAX_SPEED = 0.25;   // Not enough to overshoot
    static final double MIN_SPEED = 0.10;   // XXX This won't overcome stall
    static final int HZ = 40;

    double goalYaw = 0;

    Object poseLock = new Object();
    pose_t lastPose = null;

    private PeriodicTasks tasks = new PeriodicTasks(1);
    private class OrientTask implements PeriodicTasks.Task
    {
        public OrientTask()
        {

        }

        public void run(double dt)
        {
            DriveParams params = new DriveParams();
            params.dt = dt;
            synchronized (poseLock) {
                params.pose = lastPose;
            }
            diff_drive_t dd = drive(params);
            dd.utime = TimeUtil.utime();

            lcm.publish("DIFF_DRIVE", dd);
        }
    }

    public Orient() {}

    public Orient(HashMap<String, TypedValue> parameters)
    {
        assert (parameters.containsKey("yaw"));
        goalYaw = parameters.get("yaw").getDouble();

        if (!parameters.containsKey("no-lcm"))
            lcm.subscribe("POSE", this);

        tasks.addFixedDelay(new OrientTask(), 1.0/HZ);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.err.println("WRN: Error reading channel "+channel+": "+ex);
        }
    }

    synchronized void messageReceivedEx(LCM lcm, String channel,
            LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("POSE")) {
            pose_t msg = new pose_t(ins);
            synchronized (poseLock) {
                lastPose = msg;
            }
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
        return "ORIENT";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        Collection<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("yaw",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(-Math.PI),
                                      new TypedValue(Math.PI),
                                      true));


        return params;
    }

    /** Get a drive command from the CL. */
    public diff_drive_t drive(DriveParams params)
    {
        diff_drive_t dd = new diff_drive_t();
        dd.left_enabled = dd.right_enabled = true;
        dd.left = 0;
        dd.right = 0;

        if (params.pose == null)
            return dd;

        double yaw = LinAlg.quatPosToXYT(params.pose.orientation, params.pose.pos)[2];

        // We want to rotate toward the goal, but not overshoot.
        double dyaw = MathUtil.mod2pi(goalYaw - yaw);
        double speed = MathUtil.clamp(MAX_SPEED*(dyaw / (Math.PI/4)),
                                      MIN_SPEED,
                                      MAX_SPEED);
        if (dyaw > 0) {
            // We need to go LEFT
            dd.left = -speed;
            dd.right = speed;
        } else {
            dd.left = speed;
            dd.right = -speed;
        }

        if (Math.abs(dyaw) < Math.toRadians(2))
            dd.left = dd.right = 0;


        return dd;
    }

    public String toString()
    {
        return String.format("Orient to heading %f", goalYaw);
    }
}
