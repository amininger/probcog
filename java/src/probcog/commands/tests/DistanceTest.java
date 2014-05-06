package probcog.commands.tests;

import java.io.IOException;

import lcm.lcm.*;

import april.lcmtypes.pose_t;
import april.util.*;

import probcog.commands.TypedValue;
import probcog.lcmtypes.*;

public class DistanceTest
{
    private pose_t startPose;
    private ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);


	public DistanceTest(condition_test_t test)
    {
		super(test);
		new ListenerThread().start();
        // Save the initial pose so we can compute how far we've travelled
        System.out.println("In Distance Test");
        startPose = null;
        while(startPose == null) {
            startPose = poseCache.get();
        }
        System.out.println("Got a pose");
	}

	@Override
	protected Double getTarget(typed_value_t value)
    {
		return TypedValue.unwrapDouble(value);
	}

    /**
     * Compute how far we've travelled with respect to the original pose
     * when this test was initialized. Only use the total change in
     * pose, not integrated (is this correct?)
     *
     * @return total distance travelled
     **/
	@Override
	protected Double getValue()
    {
        pose_t pose = poseCache.get();

        if(pose == null)
            return -1.0;

        double dx = Math.abs(pose.pos[0] - startPose.pos[0]);
        double dy = Math.abs(pose.pos[1] - startPose.pos[1]);

        double dist = Math.sqrt(dx*dx +dy*dy);

		return dist;
	}

	class ListenerThread extends Thread implements LCMSubscriber {
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
