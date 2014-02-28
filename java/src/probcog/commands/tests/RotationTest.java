package probcog.commands.tests;

import java.io.IOException;

import lcm.lcm.*;

import april.lcmtypes.pose_t;
import april.jmat.*;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.commands.TypedValue;

public class RotationTest extends ConditionTest<Double>
{
    private pose_t startPose;
    private ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);


	public RotationTest(condition_test_t test)
    {
		super(test);

        // Save the initial pose so we can compute how far we've turned
		new ListenerThread().start();
        startPose = null;
        while(startPose == null) {
            startPose = poseCache.get();
        }
	}

	@Override
	protected Double getTarget(typed_value_t value)
    {
		return TypedValue.unwrapDouble(value);
	}

    /**
     * Compute how far we've turned with respect to the original pose
     * when this test was initialized. Only use the total change in
     * orientation, not integrated (is this correct?)
     *
     * @return yaw from -PI to PI; positive values indicate left turn,
     *         negative values indicate a right tun.
     **/
	@Override
	protected Double getValue()
    {
        pose_t pose = poseCache.get();

        if(pose == null)
            return -3*Math.PI;

        double[] rpyStart = LinAlg.quatToRollPitchYaw(startPose.orientation);
        double[] rpyNow = LinAlg.quatToRollPitchYaw(pose.orientation);
        double dYaw = rpyNow[2]-rpyStart[2];

		return dYaw;
	}


	class ListenerThread extends Thread implements LCMSubscriber
    {
		LCM lcm = LCM.getSingleton();

		public ListenerThread(){
			lcm.subscribe("POSE", this);
		}

		public void run(){
			while(true){
				TimeUtil.sleep(1000/60);
			}
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
}
