package probcog.commands.tests;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.lcmtypes.pose_t;
import april.jmat.*;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.commands.*;

public class RotationTest implements ConditionTest, LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    Object poseLock = new Object();
    private pose_t currPose = null;     // Most recendly recorded pose
    private pose_t lastPose = null;     // Last recorded pose
    private double yaw = 0;             // Accumulated change in yaw so far
    private double goalYaw;             // Target yaw. Must reach or exceed this target

    public void update(pose_t pose)
    {
        synchronized (poseLock) {
            lastPose = currPose;
            currPose = pose.copy();

            if (lastPose == null)
                return;
            if (currPose == null)
                return;

            double[] rpyLast = LinAlg.quatToRollPitchYaw(lastPose.orientation);
            double[] rpyNow = LinAlg.quatToRollPitchYaw(currPose.orientation);

            // Deal with wraparound.
            if (rpyLast[2]<-Math.PI/4 && rpyNow[2]>Math.PI/4) {
                rpyLast[2] += 2*Math.PI;
            } else if(rpyLast[2]>Math.PI/4 && rpyNow[2]<-Math.PI/4) {
                rpyNow[2] += 2*Math.PI;
            }

            yaw += rpyNow[2]-rpyLast[2];
        }
    }

    /** Strictly for use for parameter checking */
    public RotationTest()
    {
    }

    public RotationTest(Map<String, TypedValue> parameters)
    {
        //System.out.println("ROTATION TEST");

        assert (parameters.containsKey("yaw"));
        goalYaw = parameters.get("yaw").getDouble();

        // Hidden parameter to turn off LCM
        if (!parameters.containsKey("no-lcm"))
            lcm.subscribe("POSE", this);
    }

    public ConditionTest copyCondition()
    {
        RotationTest test = new RotationTest();
        test.goalYaw = goalYaw;

        test.lcm.subscribe("POSE", test);

        return test;
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

    /** Query whether or not the condition being tested for is currently true.
     *
     *  @return True if condition test is currently satisfied, else false
     **/
    public boolean conditionMet()
    {
        synchronized (poseLock) {
            return yaw >= goalYaw;
        }
    }

    public String toString()
    {
        return String.format("%2.2f rads", goalYaw);
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("yaw",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(0.0),
                                      new TypedValue(Math.PI),
                                      true));
        return params;
    }

    public condition_test_t getLCM()
    {
        condition_test_t ct = new condition_test_t();
        ct.name = "rotation";
        ct.num_params = 1;
        ct.param_names = new String[ct.num_params];
        ct.param_values = new typed_value_t[ct.num_params];
        ct.param_names[0] = "yaw";
        ct.param_values[0] = new TypedValue(goalYaw).toLCM();

        // Not used
        ct.compare_type = condition_test_t.CMP_GT;
        ct.compared_value = new TypedValue(0).toLCM();

        return ct;

    }
}
