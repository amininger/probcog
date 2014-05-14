package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.lcmtypes.pose_t;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.commands.*;
import probcog.robot.control.*;
import probcog.util.*;

// XXX Temporary port to new control law implementation. This is just a water-
// through-the-pipes implementation.
public class DriveForward implements ControlLaw, LCMSubscriber
{
    static final int DB_HZ = 100;
    static final double VERY_FAR = 3671000;     // Earth's radius [m]

    // XXX This needs to change
    double centerOffsetX_m = Util.getDomainConfig().requireDouble("robot.geometry.centerOffsetX_m");
    private ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);

    LCM lcm = LCM.getSingleton();

    PeriodicTasks tasks = new PeriodicTasks(1);

    private class DriveTask implements PeriodicTasks.Task
    {
        GLineSegment2D path;
        Params storedParams = Params.makeParams();

        public DriveTask()
        {
        }

        // Uses MAGIC path controller to issue drive commands to robot. These
        // seem to be stateless in this form, which means that if the
        // behavior is non-ideal, there's nothing done to correct it, but it
        // should result in approximately straight forward driving for now.
        public void run(double dt)
        {
            // Non-blocking initialization
            if (path == null) {
                pose_t initialPose = poseCache.get();
                if (initialPose == null)
                    return;

                double[] start2D = LinAlg.resize(initialPose.pos, 2);
                double[] goal2D = new double[] {start2D[0]+VERY_FAR,
                    start2D[1]+VERY_FAR};

                path = new GLineSegment2D(start2D, goal2D); // XXX - update?

                System.out.println("DriveForward ready to execute");
            }

            // Get the most recent position
            pose_t pose = poseCache.get();
            if(pose == null)
                return;
            double offset[] = LinAlg.matrixAB(LinAlg.quatToMatrix(pose.orientation),
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
        }
    }

    /** Strictly for creating instances for parameter checks */
    public DriveForward()
    {
    }

    public DriveForward(Map<String, TypedValue> parameters)
    {
        System.out.println("DRIVE FORWARD");

        lcm.subscribe("POSE", this);
        tasks.addFixedRate(new DriveTask(), 1.0/DB_HZ);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.err.println("WRN: Error receving message from channel " + channel + ": "+ex);
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

    private static void publishDiff(diff_drive_t diff_drive)
    {
        // We may get a null if there are no poses yet
        // We should throw a WRN elsewhere if that is the case
        if (diff_drive == null)
            return;

        assert(diff_drive.left <= 1 && diff_drive.left >= -1);
        assert(diff_drive.right <= 1 && diff_drive.right >= -1);

        diff_drive.utime = TimeUtil.utime();
        LCM.getSingleton().publish("DIFF_DRIVE", diff_drive);
    }

    /** Start/stop the execution of the control law.
     *
     *  @param run  True causes the control law to begin execution, false stops it
     **/
    public void setRunning(boolean run)
    {
        tasks.setRunning(run);
    }

    /** Get the name of this control law. Mostly useful for debugging purposes.
     *
     *  @return The name of the control law
     **/
    public String getName()
    {
        return "DRIVE_FORWARD";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        // No parameters, so this can just return an empty container
        return new ArrayList<TypedParameter>();
    }
}
