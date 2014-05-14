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
    Object poseLock = new Object();
    private pose_t currPose;    // Most recendly recorded pose
    private pose_t lastPose;    // Last recorded pose
    private double yaw = 0;     // Accumulated change in yaw so far
    private double goalYaw;     // Target yaw. Must reach or exceed this target


    private class UpdateThread extends Thread
    {
        public void run()
        {
            while (true) {
                synchronized (poseLock) {
                    try {
                        poseLock.wait();
                    } catch (InterruptedException ex) {}

                    if (currPose == null)
                        continue;

                    if (lastPose == null) {
                        lastPose = currPose;
                        continue;
                    }

                    double[] rpyLast = LinAlg.quatToRollPitchYaw(lastPose.orientation);
                    double[] rpyNow = LinAlg.quatToRollPitchYaw(currPose.orientation);

                    // Deal with wraparound.
                    if (rpyLast[2]<-Math.PI/4 && rpyNow[2]>Math.PI/4) {
                        rpyLast[2] += 2*Math.PI;
                    } else if(rpyLast[2]>Math.PI/4 && rpyNow[2]<-Math.PI/4) {
                        rpyNow[2] += 2*Math.PI;
                    }

                    yaw += rpyNow[2]-rpyLast[2];
                    lastPose = currPose;
                }
            }
        }
    }

    /** Strictly for use for parameter checking */
    public RotationTest()
    {
    }

    public RotationTest(Map<String, TypedValue> parameters)
    {
        System.out.println("ROTATION TEST");

        assert (parameters.containsKey("yaw"));
        goalYaw = parameters.get("yaw").getDouble();

        LCM.getSingleton().subscribe("POSE", this);
        (new UpdateThread()).start();
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
                currPose = msg;
                poseLock.notifyAll();
            }
        }
    }

    /** Query whether or not the condition being tested for is currently true.
     *
     *  @return True if condition test is currently satisfied, else false
     **/
    public boolean conditionMet()
    {
        synchronized (poseLock) {
            boolean gz0 = yaw > 0;
            boolean gz1 = goalYaw > 0;
            return (((gz1 && gz0) || !(gz0 || gz1)) && Math.abs(yaw) >= Math.abs(goalYaw));
        }
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("yaw",
                                      TypedValue.TYPE_DOUBLE));
        return params;
    }
}
