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

    static final double TURN_THRESH = Math.toRadians(15);
    static final double DIST_THRESH = 2.0;
    static final double MIN_SPEED = 0.2;    // Avoid deadband
    static final double MAX_SPEED = 0.5;
    static final double MAX_TURN = 0.3;

    private int targetID;
    private Object classyLock = new Object();
    private classification_t lastClassification = null;

    private class DriveTask implements PeriodicTasks.Task
    {
        public DriveTask()
        {
        }

        // Try to drive in the general direction of a tag, slowing down
        // when we get sufficiently close to the tag.
        public void run(double dt)
        {
            synchronized (classyLock) {
                if (lastClassification == null)
                    return;

                double dist = LinAlg.magnitude(LinAlg.resize(lastClassification.xyzrpy, 2));
                double theta = Math.atan2(lastClassification.xyzrpy[1],
                                          lastClassification.xyzrpy[0]);
                diff_drive_t dd = new diff_drive_t();
                dd.utime = TimeUtil.utime();
                dd.left_enabled = dd.right_enabled = true;
                dd.left = dd.right = 0;
                if (Math.abs(theta) > TURN_THRESH) {
                    if (theta > 0) {
                        dd.left = -MAX_TURN;
                        dd.right = MAX_TURN;
                    } else {
                        dd.left = MAX_TURN;
                        dd.right = -MAX_TURN;
                    }
                } else {
                    double mag = MathUtil.clamp(MAX_SPEED*(dist/DIST_THRESH), MIN_SPEED, MAX_SPEED);

                    dd.left = 0.3 + mag*(theta/(Math.PI/2));
                    dd.right = 0.3 + mag*(theta/(-Math.PI/2));
                }

                lcm.publish("DIFF_DRIVE", dd);
            }
        }
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
        tasks.setRunning(true);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("CLASSIFICATIONS")) {
                classification_list_t cl = new classification_list_t(ins);

                for (int i = 0; i < cl.num_classifications; i++) {
                    if (cl.classifications[i].id == targetID) {
                        synchronized (classyLock) {
                            lastClassification = cl.classifications[i];
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

    public condition_test_t getLCM()
    {
        condition_test_t ct = new condition_test_t();
        ct.name = "count";
        ct.num_params = 1;
        ct.param_names = new String[ct.num_params];
        ct.param_values = new typed_value_t[ct.num_params];
        ct.param_names[0] = "id";
        ct.param_values[0] = new TypedValue(targetID).toLCM();

        // Not used
        ct.compare_type = condition_test_t.CMP_GT;
        ct.compared_value = new TypedValue(0).toLCM();

        return ct;
    }

}
