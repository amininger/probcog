package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.commands.*;
import probcog.lcmtypes.*;

public class DriveTowardsTag implements LCMSubscriber, ControlLaw
{
    private static final double DTT_HZ = 100;
    LCM lcm = LCM.getSingleton();
    PeriodicTasks tasks = new PeriodicTasks(1);

    static final double TURN_THRESH = Math.toRadians(90);
    static final double DIST_THRESH = 2.0;
    static final double MIN_SPEED = 0.35;    // Avoid deadband
    static final double MAX_SPEED = 0.5;
    static final double MAX_TURN = 0.3;

    private int targetID;
    private Object classyLock = new Object();
    ExpiringMessageCache<classification_t> lastClassification =
        new ExpiringMessageCache<classification_t>(0.5);

    private class DriveTask implements PeriodicTasks.Task
    {
        public DriveTask()
        {
        }

        // Try to drive towards the tag, avoiding walls and obstacles as you go.
        public void run(double dt)
        {
            synchronized (classyLock) {
                DriveParams params = new DriveParams();
                params.classy = lastClassification.get();
                params.dt = dt;

                diff_drive_t dd = drive(params);
                lcm.publish("DIFF_DRIVE", dd);
            }
        }
    }

    //public diff_drive_t drive(classification_t classy, double dt)
    public diff_drive_t drive(DriveParams params)
    {
        classification_t classy = params.classy;
        double dt = params.dt;

        diff_drive_t dd = new diff_drive_t();
        dd.utime = TimeUtil.utime();
        dd.left_enabled = dd.right_enabled = true;
        dd.left = dd.right = 0;

        if (classy == null)
            return dd;

        double dist = LinAlg.magnitude(LinAlg.resize(classy.xyzrpy, 2));
        double theta = Math.atan2(classy.xyzrpy[1],
                                  classy.xyzrpy[0]);

        if (Math.abs(theta) > TURN_THRESH) {
            if (theta > 0) {
                dd.left = -MAX_TURN;
                dd.right = MAX_TURN;
            } else {
                dd.left = MAX_TURN;
                dd.right = -MAX_TURN;
            }
        } else {
            //double mag = MathUtil.clamp(MAX_SPEED*(dist/DIST_THRESH), MIN_SPEED, MAX_SPEED);

            dd.left = 0.3 + 0.6*(theta/(-Math.PI/2));
            dd.right = 0.3 + 0.6*(theta/(Math.PI/2));
        }

        return dd;
    }

    public int getID()
    {
        return targetID;
    }

    public DriveTowardsTag()
    {
    }

    public DriveTowardsTag(HashMap<String, TypedValue> parameters)
    {
        assert (parameters.containsKey("id"));
        targetID = parameters.get("id").getInt();

        lcm.subscribe("CLASSIFICATIONS", this);
        tasks.addFixedDelay(new DriveTask(), 1.0/DTT_HZ);
        //tasks.setRunning(true);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("CLASSIFICATIONS")) {
                classification_list_t cl = new classification_list_t(ins);

                for (int i = 0; i < cl.num_classifications; i++) {
                    if (cl.classifications[i].id == targetID) {
                        synchronized (classyLock) {
                            lastClassification.put(cl.classifications[i], cl.utime);
                        }
                    }
                }
            }
        } catch (IOException ex) {
            System.out.println("ERR: Couldn't handle message on channel - "+channel);
            ex.printStackTrace();
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
        return "DRIVE_TO_TAG";
    }

    public String toString()
    {
        return String.format("Drive to tag %d", targetID);
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("id",
                                      TypedValue.TYPE_INT,
                                      true));
        return params;
    }

    public control_law_t getLCM()
    {
        control_law_t cl = new control_law_t();
        cl.name = "drive-to-tag";
        cl.num_params = 1;
        cl.param_names = new String[cl.num_params];
        cl.param_values = new typed_value_t[cl.num_params];
        cl.param_names[0] = "id";
        cl.param_values[0] = new TypedValue(targetID).toLCM();

        return cl;
    }
}
