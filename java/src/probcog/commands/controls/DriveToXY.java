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

    private boolean DEBUG = false;

    // I don't think we can hit this rate. CPU intensive?
    static final double HZ = 100;

    static final double EPS = 0.05;

    static final double DISTANCE_THRESH = 0.25;
    static final double TURN_THRESH = Math.toRadians(45);
    static final double STOP_THRESH = Math.toRadians(90);
    static final double FORWARD_SPEED = 0.1;
    static final double TURN_WEIGHT = FORWARD_SPEED*20;
    static final double STALL_SPEED = 0.2; // XXX Needs some love/tuning

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

    double[] relativeXyt;
    double[] xyt = null;
    String mode = "default";
    double dist = 5.0*Util.getConfig().requireDouble("robot.geometry.radius");
    double gain = 1.0;          // Affects turn rate
    double maxSpeed = 0.4;      // Affects drive speed
    double lastTheta = Double.MAX_VALUE;

    // Determining stopping conditions, beyond just arriving near goal
    static final double MIN_RATE_TO_GOAL = 0.2; // [m/s]
    static final double GOAL_TIMEOUT = 1.0;     // [s]
    double lastDistanceToGoal = Double.MAX_VALUE;
    long lastUtime;
    Tic goalTic = new Tic();


    private class UpdateTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {
            laser_t laser = laserCache.get();
            if (laser == null) {
                return;
            }

            pose_t pose = poseCache.get();
            if (pose == null) {
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
        relativeXyt = new double[] { parameters.get("x").getDouble(),
                                     parameters.get("y").getDouble(),
                                     0.0 };

        if (parameters.containsKey("distance")) {
            dist = parameters.get("distance").getDouble();
        } else if (parameters.containsKey("mode")) {
            String mode = parameters.get("mode").toString();
            if (mode.equals("door"))
                dist = Util.getConfig().requireDouble("robot.geometry.width");
            else if (!mode.equals("default"))
                System.out.println("Unknown mode - "+mode);
        }

        if (parameters.containsKey("gain")) {
            gain = parameters.get("gain").getDouble();
        } else if (parameters.containsKey("mode")) {
            String mode = parameters.get("mode").toString();
            if (mode.equals("door"))
                gain = 4.0;
            else if (!mode.equals("default"))
                System.out.println("Unknown mode - "+mode);
        }

        if (parameters.containsKey("max-speed")) {
            maxSpeed = parameters.get("max-speed").getDouble();
        } else if (parameters.containsKey("mode")) {
            String mode = parameters.get("mode").toString();
            if (mode.equals("door"))
                maxSpeed = 0.25;
            else if (!mode.equals("default"))
                System.out.println("Unknown mode - "+mode);
        }

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
            laserCache.put(laser, TimeUtil.utime());
        } else if (poseChannel.equals(channel)) {
            pose_t pose = new pose_t(ins);
            poseCache.put(pose, TimeUtil.utime());
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
        params.add(new TypedParameter("gain",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(0.0),
                                      new TypedValue(Double.MAX_VALUE),
                                      false));
        params.add(new TypedParameter("max-speed",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(0.0),
                                      new TypedValue(1.0),
                                      false));
        HashSet<TypedValue> valid = new HashSet<TypedValue>();
        valid.add(new TypedValue("default"));
        valid.add(new TypedValue("door"));
        params.add(new TypedParameter("mode",
                                      TypedValue.TYPE_STRING,
                                      valid,
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

        if (xyt == null) {
            xyt = LinAlg.xytMultiply(robotXYT, relativeXyt);
        }

        double[] rgoal = LinAlg.xytInvMul31(robotXYT, xyt);
        double dgoal = Math.sqrt(rgoal[0]*rgoal[0] + rgoal[1]*rgoal[1]);

        if (dgoal < DISTANCE_THRESH)
            return dd;

        PotentialUtil.Params pp = new PotentialUtil.Params(params.laser,
                                                           robotXYT,
                                                           xyt);
        pp.repulsivePotential = PotentialUtil.RepulsivePotential.ALL_POINTS;
        //pp.repulsivePotential = PotentialUtil.RepulsivePotential.CLOSEST_POINT;
        pp.maxObstacleRange = dist;
        pp.fieldRes = 0.1;

        // Tie lookahead distance to speed traveled
        double maxVelocity = 2.5;   // [m/s]
        double lookaheadTime = 1.5; // [s]
        double maxLookahead = Math.max(maxVelocity*maxSpeed*lookaheadTime, 0.5);
        double shortLookahead = dgoal < maxLookahead ? 0 : MagicRobot.CHASSIS_MAIN_SIZE[0]/2;
        double longLookahead = Math.min(maxLookahead, dgoal);

        // What is the right sampling pattern? You need to look ahead to
        // maneuver early enough. You need to be careful, though, since this
        // lookahead point, once beyond your portal, will instead drag you
        // towards a wall (thus, our point just ahead of the robot).
        double[] grad = LinAlg.normalize(PotentialUtil.getGradient(new double[2], rgoal, pp));
        double[] g00 = LinAlg.normalize(PotentialUtil.getGradient(new double[] {shortLookahead, 0}, rgoal, pp));
        double[] g01 = LinAlg.normalize(PotentialUtil.getGradient(new double[] {shortLookahead+EPS,0}, rgoal, pp));
        grad = LinAlg.add(grad, LinAlg.scale(g00,1.00));
        grad = LinAlg.add(grad, LinAlg.scale(g01,0.99));

        double[] g10 = LinAlg.normalize(PotentialUtil.getGradient(new double[] {longLookahead,0}, rgoal, pp));
        double[] g11 = LinAlg.normalize(PotentialUtil.getGradient(new double[] {longLookahead+EPS,0}, rgoal, pp));
        grad = LinAlg.add(grad, LinAlg.scale(g10, 0.200));
        grad = LinAlg.add(grad, LinAlg.scale(g11, 0.199));



        ArrayList<double[]> sampleGrads = new ArrayList<double[]>();
        ArrayList<double[]> samplePoints = new ArrayList<double[]>();

        double theta = Math.atan2(grad[1], grad[0]);
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
            maxVal = minVal+1.5*pp.repulsiveWeight+1.5*pp.attractiveWeight;
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
            vvd.add(new double[] {longLookahead*Math.cos(theta), longLookahead*Math.sin(theta)});
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

        // Stopping conditions
        if (lastDistanceToGoal == Double.MAX_VALUE) {
            lastDistanceToGoal = dgoal;
            lastUtime = TimeUtil.utime();
        } else {
            long now = TimeUtil.utime();
            double dt = (now-lastUtime)/1000000.0;
            double dg = dgoal-lastDistanceToGoal;
            lastDistanceToGoal = dgoal;
            lastUtime = now;

            assert (dt != 0);
            double rate = dg/dt;

            // Make sure we're still moving towards the goal at an acctable rate
            // If not, try to stop by sending a 0 command.
            // Note that this is problematic when turning in place is necessary
            // early on.
            if (-rate > MIN_RATE_TO_GOAL) {
                goalTic.tic();
            }
            if (goalTic.toc() > GOAL_TIMEOUT) {
                return dd;
            }
        }

        // First pass at direct PWM control
        grad = LinAlg.normalize(grad);
        double speed = grad[0];
        double turn = grad[1];
        double speedLimit = maxSpeed;
        if (speed < 0) {
            speed = 0;
            int sign = turn > 0 ? 1 : -1;
            turn = sign*Math.min(.3, speedLimit);
            speedLimit = Math.abs(turn);
        }

        dd.left = speed - gain*turn;
        dd.right = speed + gain*turn;

        // Handle deadband better OR let speed controller deal with it. XXX
        double maxMag = Math.max(Math.abs(dd.left), Math.abs(dd.right));
        if (maxMag > speedLimit) {
            dd.left = speedLimit*(dd.left/maxMag);
            dd.right = speedLimit*(dd.right/maxMag);
        }

        // Refuse to drive backwards
        if (dd.left < 0 && dd.right < 0) {
            dd.left = dd.right = 0;
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
        cl.num_params = 5;
        cl.param_names = new String[cl.num_params];
        cl.param_values = new typed_value_t[cl.num_params];
        cl.param_names[0] = "x";
        cl.param_values[0] = (new TypedValue(xyt[0])).toLCM();
        cl.param_names[1] = "y";
        cl.param_values[1] = (new TypedValue(xyt[1])).toLCM();
        cl.param_names[2] = "distance";
        cl.param_values[2] = (new TypedValue(dist)).toLCM();
        cl.param_names[3] = "gain";
        cl.param_values[3] = (new TypedValue(gain)).toLCM();
        cl.param_names[4] = "max-speed";
        cl.param_values[4] = (new TypedValue(maxSpeed)).toLCM();
        cl.param_names[5] = "mode";
        cl.param_values[5] = (new TypedValue(mode)).toLCM();


        return cl;
    }

    static public void main(String[] args)
    {
        HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
        params.put("x", new TypedValue(new Double(args[0])));
        params.put("y", new TypedValue(new Double(args[1])));
        if (args.length > 2)
            params.put("distance", new TypedValue(new Double(args[2])));
        DriveToXY drive = new DriveToXY(params);
        drive.setRunning(true);
    }
}
