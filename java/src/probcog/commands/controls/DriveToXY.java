package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;

import probcog.commands.*;
import probcog.lcmtypes.*;
import probcog.util.*;

import magic2.lcmtypes.*;

public class DriveToXY implements ControlLaw, LCMSubscriber
{
    // I don't think we can hit this rate. CPU intensive?
    static final double HZ = 30;
    static final double LOOKAHEAD = 0.05;
    static final int LOOKAHEAD_STEPS = 10;

    static final double MAX_SPEED = 0.5;
    static final double FORWARD_SPEED = 0.1;
    static final double TURN_WEIGHT = 1.0;

    // XXX Get this into config
    double WHEELBASE = 0.46;
    double WHEEL_DIAMETER = 0.25;

    private PeriodicTasks tasks = new PeriodicTasks(1);
    private ExpiringMessageCache<laser_t> laserCache =
        new ExpiringMessageCache<laser_t>(.2);
    private ExpiringMessageCache<pose_t> poseCache =
        new ExpiringMessageCache<pose_t>(.2);

    LCM lcm = LCM.getSingleton();
    String laserChannel = Util.getConfig().getString("robot.lcm.laser_channel", "LASER");
    String poseChannel = Util.getConfig().getString("robot.lcm.pose_channel", "POSE");
    String driveChannel = Util.getConfig().getString("robot.lcm.drive_channel", "DIFF_DRIVE");

    double[] xyt;

    private class UpdateTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {
            laser_t laser = laserCache.get();
            if (laser == null) {
                System.out.println("ERR: No laser_t detected on channel "+laserChannel);
                return;
            }

            pose_t pose = poseCache.get();
            if (pose == null) {
                System.out.println("ERR: No pose_t detected on channel "+poseChannel);
                return;
            }

            DriveParams params = new DriveParams();
            params.pose = pose;
            params.laser = laser;
            diff_drive_t dd = drive(params);

            dd.utime = TimeUtil.utime();
            lcm.publish(driveChannel, dd);
        }
    }

    /** Strictly for use in parameter checking */
    public DriveToXY()
    {
    }

    public DriveToXY(HashMap<String, TypedValue> parameters)
    {
        assert (parameters.containsKey("x") && parameters.containsKey("y"));
        xyt = new double[] { parameters.get("x").getDouble(),
                             parameters.get("y").getDouble(),
                             0.0 };

        lcm.subscribe(laserChannel, this);
        lcm.subscribe(poseChannel, this);
        tasks.addFixedRate(new UpdateTask(), 1.0/HZ);
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
        if (laserChannel.equals(channel)) {
            laser_t laser = new laser_t(ins);
            laserCache.put(laser, laser.utime);
        } else if (poseChannel.equals(channel)) {
            pose_t pose = new pose_t(ins);
            poseCache.put(pose, pose.utime);
        }
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
        return "DRIVE_XY";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("x",
                                      TypedValue.TYPE_DOUBLE,
                                      true));
        params.add(new TypedParameter("y",
                                      TypedValue.TYPE_DOUBLE,
                                      true));

        return params;
    }

    /** Get a drive command from the CL. */
    public diff_drive_t drive(DriveParams params)
    {
        PotentialUtil.Params pp = new PotentialUtil.Params(params.laser,
                                                           params.pose,
                                                           xyt);
        pp.fieldRes = 0.025;
        //Tic tic = new Tic();
        PotentialField pf = PotentialUtil.getPotential(pp);
        //System.out.printf("%f [s]\n", tic.toc());

        diff_drive_t dd = new diff_drive_t();
        dd.left_enabled = dd.right_enabled = true;
        dd.left = dd.right = 0.0;

        // Find the gradient at our current location, then project ahead to
        // a lookahead point to compute the derivative of the gradient. Note:
        // do we really want normalized gradient values, then?
        double[] g = PotentialUtil.getGradient(new double[2], pf);
        for (int i = 1; i <= LOOKAHEAD_STEPS; i++) {
            double[] gl = PotentialUtil.getGradient(LinAlg.scale(g, i*LOOKAHEAD), pf);
            g = LinAlg.add(g, LinAlg.scale(gl, 1.0/(i+1)));
        }

        // Heading pursuit
        double theta = Math.atan2(g[1], g[0]);
        double turnSpeed = TURN_WEIGHT*(theta/Math.PI);
        double right = (2*FORWARD_SPEED + turnSpeed*WHEELBASE)/WHEEL_DIAMETER;
        double left = (2*FORWARD_SPEED - turnSpeed*WHEELBASE)/WHEEL_DIAMETER;
        double maxMag = Math.max(Math.abs(right), Math.abs(left));
        if (maxMag > 0) {
            dd.left = MAX_SPEED*(left/maxMag);
            dd.right = MAX_SPEED*(right/maxMag);
        }

        return dd;
    }

    public String toString()
    {
        return String.format("Drive to (%.1f,%.1f)", xyt[0], xyt[1]);
    }

    public control_law_t getLCM()
    {
        control_law_t cl = new control_law_t();
        cl.name = "drive-xy";
        cl.num_params = 2;
        cl.param_names = new String[cl.num_params];
        cl.param_values = new typed_value_t[cl.num_params];
        cl.param_names[0] = "x";
        cl.param_values[0] = (new TypedValue(xyt[0])).toLCM();
        cl.param_names[1] = "y";
        cl.param_values[1] = (new TypedValue(xyt[1])).toLCM();

        return cl;
    }

}
