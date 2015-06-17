package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;

import probcog.commands.*;
import probcog.util.*;

import probcog.lcmtypes.*;
import magic2.lcmtypes.*;

public class DriveTowardsTag implements LCMSubscriber, ControlLaw
{
    private static final double DTT_HZ = 30;

    LCM lcm = LCM.getSingleton();
    String mapChannel = Util.getConfig().getString("robot.lcm.map_channel", "ROBOT_MAP_DATA");
    String poseChannel = Util.getConfig().getString("robot.lcm.pose_channel", "POSE");
    String driveChannel = Util.getConfig().getString("robot.lcm.drive_channel", "DIFF_DRIVE");

    PeriodicTasks tasks = new PeriodicTasks(1);

    static final double TURN_THRESH = Math.toRadians(25);
    static final double DIST_THRESH = 2.0;
    static final double MAX_SPEED = 0.6;
    static final double MIN_SPEED = 0.25;
    static final double MAX_TURN = 0.5;

    static final double DIST_FUDGE = 0.25;
    static final double FORWARD_SPEED = 0.2;
    static final double TURN_WEIGHT = 5.0;
    static final double WHEELBASE = 0.46;
    static final double WHEEL_DIAMETER = 0.25;

    boolean sim = false;

    private int targetID;
    private String classType;
    private Object classyLock = new Object();
    private ExpiringMessageCache<classification_t> lastClassification =
        new ExpiringMessageCache<classification_t>(0.5);
    private ExpiringMessageCache<pose_t> poseCache =
        new ExpiringMessageCache<pose_t>(0.2);
    private ExpiringMessageCache<grid_map_t> gmCache =
        new ExpiringMessageCache<grid_map_t>(1.5);

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

                grid_map_t gm = gmCache.get();
                if (gm == null) {
                    return;
                }

                pose_t pose = poseCache.get();
                if (pose == null) {
                    return;
                }

                params.pose = pose;
                params.gm = gm;


                diff_drive_t dd = drive(params);
                dd.utime = TimeUtil.utime();
                lcm.publish(driveChannel, dd);
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
        double theta = Math.atan2(classy.xyzrpy[1], classy.xyzrpy[0]);

        if (dist < 0.2)
            return dd;

        double[] poseXYT = LinAlg.matrixToXYT(LinAlg.quatPosToMatrix(params.pose.orientation,
                                                                     params.pose.pos));
        double rc = Math.cos(poseXYT[2]);
        double rs = Math.sin(poseXYT[2]);
        double x = poseXYT[0] + classy.xyzrpy[0]*rc - classy.xyzrpy[1]*rs;
        double y = poseXYT[1] + classy.xyzrpy[0]*rs + classy.xyzrpy[1]*rc;

        //HashMap<String, TypedValue> typedParams = new HashMap<String, TypedValue>();
        //typedParams.put("x", new TypedValue(x));
        //typedParams.put("y", new TypedValue(y));
        //typedParams.put("alpha", new TypedValue(1.0));
        //DriveToXY driveXY = new DriveToXY(typedParams);
        //dd = driveXY.drive(params);

        //if (dd.left == 0 && dd.right == 0)
        //    return dd;
        double[] grad = new double[] { Math.cos(theta), Math.sin(theta) };
        double speed = grad[0];
        double turn = grad[1];

        dd.left = speed - turn;
        dd.right = speed + turn;

        double abs = Math.max(Math.abs(dd.left), Math.abs(dd.right));

        if (abs == 0)
            return dd;

        double maxspeed = MathUtil.clamp(dist/DIST_THRESH, 0, 1)*MAX_SPEED;
        if (abs > maxspeed) {
            dd.left = maxspeed*dd.left/abs;
            dd.right = maxspeed*dd.right/abs;
        }

        if (abs < MIN_SPEED && !sim) {
            dd.left = MIN_SPEED*dd.left/abs;
            dd.right = MIN_SPEED*dd.right/abs;
        }

        if (Math.abs(theta) > TURN_THRESH) {
            int sign = theta > 0 ? 1 : -1;
            dd.left = -sign*MAX_TURN;
            dd.right = sign*MAX_TURN;
        }

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

        if (parameters.containsKey("sim"))
            sim = true;

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
            } else if (mapChannel.equals(channel)) {
                robot_map_data_t rmd = new robot_map_data_t(ins);
                gmCache.put((grid_map_t)rmd.gridmap.copy(), rmd.utime);
            } else if (poseChannel.equals(channel)) {
                pose_t pose = new pose_t(ins);
                poseCache.put(pose, pose.utime);
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
        if (run) {
            lcm.subscribe(mapChannel, this);
            lcm.subscribe(poseChannel, this);
            lcm.subscribe("CLASSIFICATIONS", this);
        } else {
            lcm.unsubscribe(mapChannel, this);
            lcm.subscribe(poseChannel, this);
            lcm.unsubscribe("CLASSIFICATIONS", this);
        }
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
