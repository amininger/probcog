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
    private pose_t lastPose;
    private double yaw;

	public RotationTest(condition_test_t test)
    {
		super(test);
		new ListenerThread().start();
        yaw = 0;
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
        System.out.println(yaw);
		return yaw;
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

                if(lastPose != null) {
                    double[] rpyLast = LinAlg.quatToRollPitchYaw(lastPose.orientation);
                    double[] rpyNow = LinAlg.quatToRollPitchYaw(msg.orientation);

                    if(rpyLast[2]<-Math.PI/4 && rpyNow[2]>Math.PI/4)
                    {
                        rpyLast[2] += 2*Math.PI;
                    }
                    else if(rpyLast[2]>Math.PI/4 && rpyNow[2]<-Math.PI/4)
                    {
                        rpyNow[2] += 2*Math.PI;
                    }
                    yaw += rpyNow[2]-rpyLast[2];
                }

                lastPose = msg;
            }
        }
    }
}
