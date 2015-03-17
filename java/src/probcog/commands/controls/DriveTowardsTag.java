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

    static final double DIST_FUDGE = 0.25;
    static final double FORWARD_SPEED = 0.2;
    static final double TURN_WEIGHT = 5.0;
    static final double WHEELBASE = 0.46;
    static final double WHEEL_DIAMETER = 0.25;

    private int targetID;
    private String classType;
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

    public void handleClassification(classification_t curr, long utime)
    {
        if (!curr.name.equals(classType))
            return;
        synchronized (classyLock) {
            if (lastClassification.get() == null) {
                lastClassification.put(curr, utime);
            }
            classification_t classy = lastClassification.get();
            if (curr.id == classy.id) {
                lastClassification.put(curr, utime);
                return;
            }

            double d0 = LinAlg.sq(classy.xyzrpy[0]) + LinAlg.sq(classy.xyzrpy[1]);
            double d1 = LinAlg.sq(curr.xyzrpy[0]) + LinAlg.sq(curr.xyzrpy[1]);
            if (d1+DIST_FUDGE < d0) {
                lastClassification.put(curr, Math.max(utime, classy.utime+1));
                return;
            }
        }
    }

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

            //dd.left = 0.3 + 0.6*(theta/(-Math.PI/2));
            //dd.right = 0.3 + 0.6*(theta/(Math.PI/2));
            double turn_speed = TURN_WEIGHT*(theta/Math.PI);
            dd.right = (2*FORWARD_SPEED + turn_speed*WHEELBASE)/WHEEL_DIAMETER;
            dd.left = (2*FORWARD_SPEED - turn_speed*WHEELBASE)/WHEEL_DIAMETER;
            double max_mag = Math.max(Math.abs(dd.left), Math.abs(dd.right));

            if (max_mag > 0) {
                dd.left = MAX_SPEED*dd.left/max_mag;
                dd.right = MAX_SPEED*dd.right/max_mag;
            }
        }

        if (dist <= 0.2)
            dd.left = dd.right = 0;


        return dd;
    }

    public int getID()
    {
        return targetID;
    }

    public String getClassType()
    {
        return classType;
    }

    public DriveTowardsTag()
    {
    }

    public DriveTowardsTag(HashMap<String, TypedValue> parameters)
    {
        //assert (parameters.containsKey("id"));
        //targetID = parameters.get("id").getInt();
        assert (parameters.containsKey("class"));
        classType = parameters.get("class").toString();

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
                    classification_t curr = cl.classifications[i];
                    handleClassification(curr, cl.utime);
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
        return String.format("Drive to tag %s", classType);
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        //params.add(new TypedParameter("id",
        //                              TypedValue.TYPE_INT,
        //                              true));
        params.add(new TypedParameter("class",
                                      TypedValue.TYPE_STRING,
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
        cl.param_names[0] = "class";
        cl.param_values[0] = new TypedValue(classType).toLCM();

        return cl;
    }
}
