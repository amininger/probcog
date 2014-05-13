package probcog.commands.tests;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.commands.*;
import probcog.lcmtypes.*;

public class DistanceTest implements ConditionTest, LCMSubscriber
{
    static final int DT_HZ = 30;

    LCM lcm = LCM.getSingleton();

    Object poseLock = new Object();
    private ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);
    private pose_t startPose = null;
    private double goalDistance = 0;
    private double currentDistance = 0;

    private class UpdateTask extends Thread
    {
        public void run()
        {
            // Initialize
            while (startPose == null) {
                synchronized (poseLock) {
                    try {
                        poseLock.wait();
                    } catch (InterruptedException ex) {}
                    startPose = poseCache.get();
                }
            }

            while (true) {
                // Wait for LCM
                synchronized (poseLock) {
                    try {
                        poseLock.wait();
                    } catch (InterruptedException ex) {}

                    pose_t currPose = poseCache.get();  // XXX This cache seems silly now

                    // Update distance
                    currentDistance = LinAlg.distance(startPose.pos,
                                                      currPose.pos,
                                                      2);
                }
            }
        }
    }

    /** Strictly for use for parameter checking */
    public DistanceTest()
    {
    }

    public DistanceTest(Map<String, TypedValue> parameters)
    {
        System.out.println("DISTANCE TEST");

        assert (parameters.containsKey("distance"));
        goalDistance = parameters.get("distance").getDouble();

        lcm.subscribe("POSE", this);
        (new UpdateTask()).start();
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
                poseCache.put(msg, msg.utime);
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
            return goalDistance < currentDistance;
        }
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("distance",
                                      TypedValue.TYPE_DOUBLE));
        return params;
    }
}
