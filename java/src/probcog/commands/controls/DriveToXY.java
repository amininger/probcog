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

// XXX DEBUG
import java.awt.*;
import java.awt.image.*;
import javax.swing.*;
import april.vis.*;

public class DriveToXY implements ControlLaw, LCMSubscriber
{
    VisWorld vw;

    private boolean DEBUG = true;

    // I don't think we can hit this rate. CPU intensive?
    static final double HZ = 30;

    static final double DISTANCE_THRESH = 0.25;
    static final double TURN_THRESH = Math.toRadians(90);
    static final double MAX_SPEED = 0.5;
    static final double TURN_SPEED = 0.4;
    static final double FORWARD_SPEED = 0.1;
    static final double TURN_WEIGHT = 1.0;

    // XXX Get this into config
    double WHEELBASE = 0.46;
    double WHEEL_DIAMETER = 0.25;
    double REAR_AXLE_OFFSET = 0.20;

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
    double lastTheta = Double.MAX_VALUE;

    private class UpdateTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {
            laser_t laser = laserCache.get();
            if (laser == null) {
                //System.err.println("ERR: No laser_t detected on channel "+laserChannel);
                return;
            }

            pose_t pose = poseCache.get();
            if (pose == null) {
                //System.err.println("ERR: No pose_t detected on channel "+poseChannel);
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

        tasks.addFixedRate(new UpdateTask(), 1.0/HZ);

        if (DEBUG) {
            JFrame jf = new JFrame("Debug DriveXY");
            jf.setSize(400, 400);
            jf.setLayout(new BorderLayout());

            vw = new VisWorld();
            VisLayer vl = new VisLayer(vw);
            VisCanvas vc = new VisCanvas(vl);
            jf.add(vc);

            jf.setVisible(true);
        }
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
        if (run) {
            lcm.subscribe(laserChannel, this);
            lcm.subscribe(poseChannel, this);
        } else {
            lcm.unsubscribe(laserChannel, this);
            lcm.unsubscribe(poseChannel, this);
        }
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
        diff_drive_t dd = new diff_drive_t();
        dd.left_enabled = dd.right_enabled = true;
        dd.left = dd.right = 0.0;

        // Project pose to robot center. Stop if close to goal.
        double[] robotXYT = LinAlg.matrixToXYT(LinAlg.quatPosToMatrix(params.pose.orientation,
                                                                      params.pose.pos));
        double[] rgoal = LinAlg.xytInvMul31(robotXYT, xyt);
        double dgoal = Math.sqrt(rgoal[0]*rgoal[0] + rgoal[1]*rgoal[1]);

        if (LinAlg.distance(robotXYT, xyt, 2) < DISTANCE_THRESH)
            return dd;

        pose_t pose = new pose_t();
        pose.orientation = LinAlg.copy(params.pose.orientation);
        pose.pos = new double[] {robotXYT[0], robotXYT[1], 0};

        PotentialUtil.Params pp = new PotentialUtil.Params(params.laser,
                                                           pose,
                                                           xyt);
        pp.fieldRes = 0.1;
        if (params.pp != null)
            pp = params.pp;

        ArrayList<double[]> sampleGrads = new ArrayList<double[]>();
        ArrayList<double[]> samplePoints = new ArrayList<double[]>();
        double theta = -Math.PI;

        double[] grad = new double[2];

        //Tic tic = new Tic();
        double[] ys = new double[] {-.4, -.3, -.2, -.1, 0, .1, .2, .3, .4};
        double[] xs = new double[] {0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1, 1.1, 1.2, 1.3, 1.4, 1.5};
        for (double y: ys) {
            for (double x: xs) {
                double[] rxy = new double[] {x, y};
                if (LinAlg.magnitude(rxy) > dgoal)
                    continue;
                double[] g0 = PotentialUtil.getGradient(rxy, rgoal, pp);
                if (LinAlg.magnitude(g0) == 0)
                    continue;
                g0 = LinAlg.normalize(g0);
                //g0 = LinAlg.scale(g0, Math.abs(y)+.2/x);

                if (DEBUG) {
                    samplePoints.add(rxy);
                    sampleGrads.add(rxy);
                    sampleGrads.add(LinAlg.add(rxy, LinAlg.scale(g0, 0.1)));
                }
                LinAlg.plusEquals(grad, g0);
            }
        }
        //System.out.printf("%f [s]\n", tic.toc());

        if (MathUtil.doubleEquals(grad[0], 0) && MathUtil.doubleEquals(grad[1], 0)) {
            return dd;
        }
        theta = Math.atan2(grad[1], grad[0]);
        grad = LinAlg.normalize(grad);

        if (DEBUG) {
            // Render the field
            PotentialField pf = PotentialUtil.getPotential(pp);
            int[] map = new int[] {0xffffff00,
                0xffff00ff,
                0x0007ffff,
                0xff0000ff,
                0xff2222ff};
            double minVal = pf.getMinValue();
            ColorMapper cm = new ColorMapper(map, minVal, 2.5*minVal);

            VisWorld.Buffer vb = vw.getBuffer("laser");
            vb.setDrawOrder(100);
            ArrayList<double[]> plaser = new ArrayList<double[]>();
            for (int i = 0; i < params.laser.nranges; i++) {
                double r = params.laser.ranges[i];
                if (r < 0)
                    continue;
                double t = params.laser.rad0 + i*params.laser.radstep;
                plaser.add(new double[]{r*Math.cos(t), r*Math.sin(t)});
            }
            vb.addBack(new VzPoints(new VisVertexData(plaser),
                                    new VzPoints.Style(Color.yellow, 3)));
            vb.swap();

            vb = vw.getBuffer("pf");
            vb.addBack(pf.getVisObject(cm));
            vb.swap();

            vb = vw.getBuffer("samples");
            if (samplePoints.size() > 0) {
                VisVertexData vvd = new VisVertexData();
                vvd.add(new double[2]);
                vvd.add(grad);
                vb.addBack(new VzLines(vvd, VzLines.LINES, new VzLines.Style(Color.gray, 3)));
                vb.addBack(new VzPoints(new VisVertexData(samplePoints),
                                        new VzPoints.Style(Color.green, 3)));
                vb.addBack(new VzLines(new VisVertexData(sampleGrads),
                                       VzLines.LINES,
                                       new VzLines.Style(Color.gray, 1)));
            }
            vb.swap();
        }

        // Determine if we're stuck. Not perfect...we can still end up oscillating
        if (lastTheta != Double.MAX_VALUE) {
            double dtheta = MathUtil.mod2pi(lastTheta - theta);
            if (Math.abs(dtheta) > Math.PI/2)
                return dd;
        }
        lastTheta = theta;

        if (Math.abs(theta) > TURN_THRESH) {
            if (theta > 0) {
                dd.left = -TURN_SPEED;
                dd.right = TURN_SPEED;
            } else {
                dd.left = TURN_SPEED;
                dd.right = -TURN_SPEED;
            }
        } else {
            double turnSpeed = TURN_WEIGHT*(theta/Math.PI);
            double right = (2*FORWARD_SPEED + turnSpeed*WHEELBASE)/WHEEL_DIAMETER;
            double left = (2*FORWARD_SPEED - turnSpeed*WHEELBASE)/WHEEL_DIAMETER;
            double maxMag = Math.max(Math.abs(right), Math.abs(left));
            if (maxMag > 0) {
                dd.left = MAX_SPEED*(left/maxMag);
                dd.right = MAX_SPEED*(right/maxMag);
            }
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
