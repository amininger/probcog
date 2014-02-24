package probcog.commands.controls;

import java.io.IOException;

import lcm.lcm.*;

import april.util.*;

import probcog.lcmtypes.*;
import probcog.commands.TypedValue;

public class Turn extends ControlLaw
{
    static final int DD_BCAST_PERIOD_MS = 33; //XXX - Should go in config

    static LCM lcm = LCM.getSingleton();

	enum Direction { LEFT, RIGHT };
	Direction dir;

    private ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);
    private pose_t initialPose;

	public Turn(control_law_t controlLaw)
    {
		super(controlLaw);

		dir = Direction.LEFT;

		// Parameters:
		//   direction = { left, right }
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("direction")){
				String value = TypedValue.unwrapString(controlLaw.param_values[p]);
				if(value.equals("left")){
					dir = Direction.LEFT;
				} else if(value.equals("right")){
					dir = Direction.RIGHT;
				}
			}
		}
	}

	@Override
	public void execute()
    {
        initialPose = null;
        while(initialPose == null) {
            initialPose = poseCache.get();
        }

        boolean turn = getStatus().equals(ControlLaw.Status.EXECUTING);
        while (turn) {

            TimeUtil.sleep(DD_BCAST_PERIOD_MS);

            // Get the most recent position
            pose_t pose = poseCache.get();
            if(pose == null)
                continue;

            // Initialize diff drive with no movement
            diff_drive_t dd = new diff_drive_t();
            dd.left_enabled = dd.right_enabled = true;
            dd.left = 0;
            dd.right = 0;

            // Change left and right wheels depending on turn direction
            if (dir.equals(Direction.RIGHT)) {
                dd.left = -0.1;
                dd.right = 0.1;
            }
            else if (dir.equals(Direction.LEFT)) {
                dd.left = 0.1;
                dd.right = -0.1;
            }

            publishDiff(dd);

            // Test current status to determine whether to stop driving
            turn = getStatus().equals(ControlLaw.Status.EXECUTING);
        }
	}

	@Override
	public ControlLaw.Status getStatus()
    {
		// TODO: may return EARLY_TERM or FAILURE if
		//   something went wrong

		if(termCond.evaluate() == true){
			return ControlLaw.Status.FINISHED;
		}
		return ControlLaw.Status.EXECUTING;
	}

    // XXX publishDiff is in two classes - can we consolidate?
    private void publishDiff(diff_drive_t diff_drive)
    {
        // We may get a null if there are no poses yet
        // We should throw a WRN elsewhere if that is the case
        if (diff_drive == null)
            return;

        assert(diff_drive.left <= 1 && diff_drive.left >= -1);
        assert(diff_drive.right <= 1 && diff_drive.right >= -1);

        diff_drive.utime = TimeUtil.utime();
        lcm.publish("DIFF_DRIVE", diff_drive);
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
