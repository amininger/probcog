package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.util.*;
import april.jmat.*;

import probcog.commands.*;

import probcog.lcmtypes.*;
import magic2.lcmtypes.*;

public class Turn implements ControlLaw, LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    static final int DD_HZ = 100;
    static final double ROBOT_SPEED = 0.5;
    static final double SIM_SPEED = 0.25;

    Object poseLock = new Object();
    pose_t currPose = null;
    pose_t lastPose = null;

    boolean sim = false;

    private double yaw = 0;
    private double goalYaw = Double.MAX_VALUE;
	enum Direction { LEFT, RIGHT };
	Direction dir;

    private PeriodicTasks tasks = new PeriodicTasks(1);

    private class TurnTask implements PeriodicTasks.Task
    {
        public TurnTask()
        {
            //System.out.println("Turn ready to execute");
        }

        public void run(double dt)
        {
            DriveParams params = new DriveParams();
            params.dt = dt;
            diff_drive_t dd = drive(params);

            publishDiff(dd);
        }
    }

    public void update(pose_t pose)
    {
        synchronized (poseLock) {
            lastPose = currPose;
            currPose = pose.copy();

            if (lastPose == null) {
                return;
            }
            assert (lastPose != null);

            double[] rpyLast = LinAlg.quatToRollPitchYaw(lastPose.orientation);
            double[] rpyNow = LinAlg.quatToRollPitchYaw(currPose.orientation);

            // Deal with wraparound.
            if (rpyLast[2]<-Math.PI/4 && rpyNow[2]>Math.PI/4) {
                rpyLast[2] += 2*Math.PI;
            } else if(rpyLast[2]>Math.PI/4 && rpyNow[2]<-Math.PI/4) {
                rpyNow[2] += 2*Math.PI;
            }

            yaw += Math.abs(rpyNow[2]-rpyLast[2]);
        }
    }

    //public diff_drive_t drive(double dt)
    public diff_drive_t drive(DriveParams params)
    {
        double dt = params.dt;

        // Initialize diff drive with no movement
        diff_drive_t dd = new diff_drive_t();
        dd.left_enabled = dd.right_enabled = true;
        dd.left = 0;
        dd.right = 0;

        double speed = ROBOT_SPEED;
        if (sim)
            speed = SIM_SPEED;

        // Change left and right wheels depending on turn direction
        if (dir.equals(Direction.RIGHT)) {
            dd.left = speed;
            dd.right = -speed;
        }
        else if (dir.equals(Direction.LEFT)) {
            dd.left = -speed;
            dd.right = speed;
        }

        return dd;
    }

    public void reset()
    {
        currPose = null;
        lastPose = null;
        yaw = 0;
    }

    /** Strictly for use for parameter checking */
    public Turn()
    {
    }

    public Turn(Map<String, TypedValue> parameters)
    {
        // Needs a direction to turn, currently
        if (parameters.containsKey("direction")){
        	int direction = parameters.get("direction").getInt();
            if (direction > 0) // CW
                dir = Direction.LEFT;
            else // CCW
                dir = Direction.RIGHT;
        } else {
        	dir = Direction.LEFT;
        }

        if (parameters.containsKey("yaw"))
            goalYaw = Math.abs(parameters.get("yaw").getDouble());

        if (parameters.containsKey("sim"))
            sim = true;

        tasks.addFixedDelay(new TurnTask(), 1.0/DD_HZ);
    }

    /** Start/stop the execution of the control law.
     *
     *  @param run  True causes the control law to begin execution, false stops it
     **/
    public void setRunning(boolean run)
    {
        if (run) {
            // no-lcm?
            lcm.subscribe("POSE", this);
        } else {
            lcm.unsubscribe("POSE", this);
        }
        tasks.setRunning(run);
    }

    /** Get the name of this control law. Mostly useful for debugging purposes.
     *
     *  @return The name of the control law
     **/
    public String getName()
    {
        return "TURN";
    }

    public String toString()
    {
        return String.format("Turn %s", dir == Direction.LEFT ? "LEFT" : "RIGHT");
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        ArrayList<TypedValue> options = new ArrayList<TypedValue>();
        options.add(new TypedValue(-1));
        options.add(new TypedValue(1));
        params.add(new TypedParameter("direction",
                                      TypedValue.TYPE_INT,
                                      options,
                                      true));
        params.add(new TypedParameter("yaw",
                                      TypedValue.TYPE_DOUBLE,
                                      false));
        return params;
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
            update(msg);
        }
    }

    public control_law_t getLCM()
    {
        control_law_t cl = new control_law_t();
        cl.name = "turn";
        cl.num_params = 2;
        cl.param_names = new String[cl.num_params];
        cl.param_values = new typed_value_t[cl.num_params];
        cl.param_names[0] = "direction";
        cl.param_values[0] = new TypedValue(dir == Direction.LEFT ? 1 : -1).toLCM();
        cl.param_names[1] = "yaw";
        cl.param_values[1] = new TypedValue(goalYaw).toLCM();

        return cl;
    }

    // XXX publishDiff is in two classes - can we consolidate?
    static private void publishDiff(diff_drive_t diff_drive)
    {
        // We may get a null if there are no poses yet
        // We should throw a WRN elsewhere if that is the case
        if (diff_drive == null)
            return;

        assert(diff_drive.left <= 1 && diff_drive.left >= -1);
        assert(diff_drive.right <= 1 && diff_drive.right >= -1);

        diff_drive.utime = TimeUtil.utime();
        LCM.getSingleton().publish("DIFF_DRIVE", diff_drive);
    }
}
