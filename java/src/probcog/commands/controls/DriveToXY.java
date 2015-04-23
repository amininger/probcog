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
    static final double HZ = 100;

    static final double EPS = 0.0001;

    // Integration method parameters
    static final double LOOKAHEAD_M = 1.0;
    static final double LOOKAHEAD_STEP_M = 0.5;
    static final double THETA_RANGE_RAD = Math.abs(3*Math.PI/4);
    static final double THETA_STEP = Math.toRadians(2.5);


    static final double DISTANCE_THRESH = 0.25;
    static final double TURN_THRESH = Math.toRadians(45);
    static final double STOP_THRESH = Math.toRadians(90);
    static final double FORWARD_SPEED = 0.1;
    static final double TURN_WEIGHT = FORWARD_SPEED*20;
    static final double STALL_SPEED = 0.2;

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
    double dist = 2*Util.getConfig().requireDouble("robot.geometry.radius");
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

        if (parameters.containsKey("distance"))
            dist = parameters.get("distance").getDouble();

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
        params.add(new TypedParameter("distance",
                                      TypedValue.TYPE_DOUBLE,
                                      false));

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
        robotXYT[0] += MagicRobot.CENTER_X_OFFSET*Math.cos(robotXYT[2]);
        robotXYT[1] += MagicRobot.CENTER_X_OFFSET*Math.sin(robotXYT[2]);


        double[] rgoal = LinAlg.xytInvMul31(robotXYT, xyt);
        double dgoal = Math.sqrt(rgoal[0]*rgoal[0] + rgoal[1]*rgoal[1]);

        if (dgoal < DISTANCE_THRESH)
            return dd;

        PotentialUtil.Params pp = new PotentialUtil.Params(params.laser,
                                                           robotXYT,
                                                           xyt);
        pp.repulsivePotential = PotentialUtil.RepulsivePotential.ALL_POINTS;
        pp.maxObstacleRange = dist;
        pp.fieldRes = 0.1;

        double[] grad = new double[2];
        double[][] pts = new double[][] {{0,0}, {0.3, 0}};
        double[] offset = new double[] {EPS, 0};
        for (double[] pt: pts) {
            double[] g0 = LinAlg.normalize(PotentialUtil.getGradient(pt, rgoal, pp));
            grad = LinAlg.add(grad, g0);
        }

        ArrayList<double[]> sampleGrads = new ArrayList<double[]>();
        ArrayList<double[]> samplePoints = new ArrayList<double[]>();

        double theta = -Math.PI;
        //double minIntegral = Double.MAX_VALUE;

        //for (double t = -THETA_RANGE_RAD; t <= THETA_RANGE_RAD; t += THETA_STEP) {
        //    double integral = 0;
        //    for (int i = 1; i < Math.ceil(LOOKAHEAD_M/LOOKAHEAD_STEP_M); i++) {
        //        double rx = Math.cos(t)*i*LOOKAHEAD_STEP_M;
        //        double ry = Math.sin(t)*i*LOOKAHEAD_STEP_M;
        //        integral += PotentialUtil.getRelative(rx, ry, rgoal, pp);
        //    }

        //    if (integral < minIntegral) {
        //        theta = t;
        //        minIntegral = integral;
        //    }
        //}
        //double[] grad = new double[2];
        //Tic tic = new Tic();
        //double[] ys = new double[] {-.4, -.3, -.2, -.1, 0, .1, .2, .3, .4};
        //double[] xs = new double[] {0, .1, .2, .3, .4, .5, .6, .7, .8, .9, 1, 1.1, 1.2};
        //for (double y: ys) {
        //    for (double x: xs) {
        //        double[] rxy = new double[] {x, y};
        //        double rmag = LinAlg.magnitude(rxy);
        //        if (rmag > 1.25)
        //            continue;

        //        if (rmag > dgoal)
        //            continue;
        //        double[] g0 = PotentialUtil.getGradient(rxy, rgoal, pp);
        //        double gmag = LinAlg.magnitude(g0);
        //        if (gmag == 0)
        //            continue;
        //        g0[0] /= gmag;
        //        g0[1] /= gmag;
        //        //g0 = LinAlg.scale(g0, Math.pow(.1/Math.max(0.1, rmag), .5));

        //        if (DEBUG) {
        //            samplePoints.add(rxy);
        //            sampleGrads.add(rxy);
        //            sampleGrads.add(LinAlg.add(rxy, LinAlg.scale(g0, 0.1)));
        //        }
        //        LinAlg.plusEquals(grad, g0);
        //    }
        //}
        //System.out.printf("%f [s]\n", tic.toc());

        //if (MathUtil.doubleEquals(grad[0], 0) && MathUtil.doubleEquals(grad[1], 0)) {
        //    return dd;
        //}
        theta = Math.atan2(grad[1], grad[0]);
        //grad = LinAlg.normalize(grad);

        if (DEBUG) {
            // Render the field
            PotentialField pf = PotentialUtil.getPotential(pp);
            int[] map = new int[] {0xffffff00,
                0xffff00ff,
                0x0007ffff,
                0xff0000ff,
                0xff2222ff};
            double minVal = pf.getMinValue();
            double maxVal = pf.getMaxValue();
            maxVal = Math.max(minVal+pp.repulsiveWeight+pp.attractiveWeight, 5*minVal);
            ColorMapper cm = new ColorMapper(map, minVal, maxVal);

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
            VisVertexData vvd = new VisVertexData();
            vvd.add(new double[2]);
            vvd.add(new double[] {LOOKAHEAD_M*Math.cos(theta), LOOKAHEAD_M*Math.sin(theta)});
            vb.addBack(new VzLines(vvd, VzLines.LINES, new VzLines.Style(Color.gray, 3)));
            if (samplePoints.size() > 0) {
                vb.addBack(new VzPoints(new VisVertexData(samplePoints),
                                        new VzPoints.Style(Color.green, 3)));
                vb.addBack(new VzLines(new VisVertexData(sampleGrads),
                                       VzLines.LINES,
                                       new VzLines.Style(Color.gray, 1)));
            }
            vb.swap();
        }

        if (theta == -Math.PI)
            return dd;

        // Determine if we're stuck. Not perfect...we can still end up oscillating
        if (lastTheta != Double.MAX_VALUE) {
            double dtheta = MathUtil.mod2pi(lastTheta - theta);
            if (Math.abs(dtheta) > STOP_THRESH)
                return dd;
        }
        lastTheta = theta;

        if (Math.abs(theta) > TURN_THRESH) {
            double turnSpeed = Math.max(0.3, params.maxSpeed);
            if (theta > 0) {
                dd.left = -turnSpeed;
                dd.right = turnSpeed;
            } else {
                dd.left = turnSpeed;
                dd.right = -turnSpeed;
            }
        } else {
            double turnSpeed = TURN_WEIGHT*(theta/Math.PI);
            double right = (2*FORWARD_SPEED + turnSpeed*WHEELBASE)/WHEEL_DIAMETER;
            double left = (2*FORWARD_SPEED - turnSpeed*WHEELBASE)/WHEEL_DIAMETER;
            double maxMag = Math.max(Math.abs(right), Math.abs(left));
            if (maxMag > params.maxSpeed) {
                dd.left = params.maxSpeed*(left/maxMag);
                dd.right = params.maxSpeed*(right/maxMag);
            }
            if (Math.abs(dd.left) < STALL_SPEED)
                dd.left = 0;
            if (Math.abs(dd.right) < STALL_SPEED)
                dd.right = 0;
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
