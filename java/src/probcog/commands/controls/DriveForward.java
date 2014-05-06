package probcog.commands.controls;

import java.io.IOException;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.lcmtypes.pose_t;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.robot.control.*;
import probcog.util.Util;

public class DriveForward extends ControlLaw
{
    static final int DD_BCAST_PERIOD_MS = 33; // 30 Hz
    static final double VERY_FAR = 6371000; // meters in Earth radius

    double centerOffsetX_m = Util.getConfig().requireDouble("robot.geometry.centerOffsetX_m");

    static LCM lcm = LCM.getSingleton();
    private ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);

    private pose_t initialPose;

    public DriveForward(control_law_t controlLaw)
    {
        super(controlLaw);
		new ListenerThread().start();
    }

    @Override
    public void execute()
    {
        System.out.println("Executing drive-forward");

        initialPose = null;
        while(initialPose == null) {
            initialPose = poseCache.get();
        }

        double[] start2D = LinAlg.resize(initialPose.pos, 2);
        double[] goal2D = new double[]{start2D[0]+VERY_FAR,
                                       start2D[1]+VERY_FAR};
        GLineSegment2D path = new GLineSegment2D(start2D, goal2D); // XXX - update?
        Params storedParams = Params.makeParams();

        boolean drive = getStatus().equals(ControlLaw.Status.EXECUTING);

        while (drive) {
            TimeUtil.sleep(DD_BCAST_PERIOD_MS);

            // Get the most recent position
            pose_t pose = poseCache.get();
            if(pose == null)
                continue;
            double offset[]  = LinAlg.matrixAB(LinAlg.quatToMatrix(pose.orientation),
                                               new double[] {centerOffsetX_m, 0 , 0, 1});
            double center_pos[] = new double[]{pose.pos[0] + offset[0],
                                               pose.pos[1] + offset[1] };

            // Create and publish controls used by RobotDriver
            diff_drive_t dd = PathControl.getDiffDrive(center_pos,
                                                       pose.orientation,
                                                       path,
                                                       storedParams,
                                                       1.0);
            publishDiff(dd);

            // Test current status to determine whether to stop driving
            drive = getStatus().equals(ControlLaw.Status.EXECUTING);
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
