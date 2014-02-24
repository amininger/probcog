package probcog.commands.tests;

import java.io.IOException;

import lcm.lcm.*;

import april.util.*;

import probcog.commands.TypedValue;
import probcog.lcmtypes.*;

public class DistanceTest extends ConditionTest<Double> implements LCMSubscriber
{
    private pose_t initialPose;
    private ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);


	public DistanceTest(condition_test_t test)
    {
		super(test);

        // Save the initial pose so we can compute how far we've travelled
        initialPose = null;
        while(initialPose == null) {
            initialPose = poseCache.get();
        }
	}

	@Override
	protected Double getTarget(typed_value_t value)
    {
		return TypedValue.unwrapDouble(value);
	}

	@Override
	protected Double getValue()
    {
        pose_t pose = poseCache.get();

        // XXX - How can we report an error from here?
        if(pose == null)
            return -1.0;

        double dx = Math.abs(pose.pos[0] - initialPose.pos[0]);
        double dy = Math.abs(pose.pos[1] - initialPose.pos[1]);

        double dist = Math.sqrt(dx*dx +dy*dy);

        // XXX - Not sure this should be evaluated in a <=, ==, >= fashion
		return dist;
	}

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }

    synchronized void messageReceivedEx(LCM lcm, String channel,
                           LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("POSE")) {
            pose_t msg = new pose_t(ins);
            poseCache.put(msg, msg.utime);
        }
    }
}
