package probcog.sim;

import java.awt.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;
import java.awt.event.*;
import java.awt.*;
import javax.swing.*;

import lcm.lcm.*;

import april.config.Config;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.jmat.geom.*;
import april.vis.*;
import april.lcm.*;
import april.util.*;
import april.sim.*;
import april.lcmtypes.*;

import probcog.classify.TagClassifier;
import probcog.commands.CommandInterpreter;
import probcog.lcmtypes.*;
import probcog.util.*;
import probcog.vis.*;
import probcog.perception.ObstacleMap;
import probcog.robot.control.*;
import probcog.sensor.SimKinectSensor;

public class SimRobot implements SimObject, LCMSubscriber
{
    static Random classifierRandom = new Random(3611871);

    int ROBOT_ID = 6;
    SimWorld sw;
    DifferentialDrive drive;

    boolean useCoarseShape;
    boolean useNoise;
    boolean drawSensor;
    int robotID;

    LCM lcm = LCM.getSingleton();

    CompoundShape shape;
    VisObject visObj;

    ExpiringMessageCache<diff_drive_t> diffdriveCache = new ExpiringMessageCache<diff_drive_t>(0.25);
    ExpiringMessageCache<gamepad_t> gamepadCache = new ExpiringMessageCache<gamepad_t>(0.25);

    PeriodicTasks tasks = new PeriodicTasks(2);

    // static variables ..
    static Random r = new Random();
    static Model4 model4 = new Model4();

    public SimRobot(SimWorld sw)
    {
        this.sw = sw;
        // XXX These don't exist?
        useCoarseShape = sw.config.getBoolean("simulator.sim_magic_robot.use_coarse_shape", true);
        useNoise = sw.config.getBoolean("simulator.sim_magic_robot.use_noise", false);
        drawSensor = sw.config.getBoolean("simulator.sim_magic_robot.draw_sensor", false);
        this.robotID = ROBOT_ID;

        shape = makeShape();

        if (drawSensor) {
            visObj = new VisChain(model4, Model4.Model4Head(Util.getConfig(), 0, 0, 0));
        } else
            visObj = model4;

        // visObj = new VzSphere(.5, new VzMesh.Style(Color.RED));

        CommandInterpreter ci = new CommandInterpreter();

        drive = new DifferentialDrive(sw, this, new double[3]);
        drive.centerOfRotation = new double[] { 0.13, 0, 0 };

        lcm.subscribe("GAMEPAD", this);
        lcm.subscribe("DIFF_DRIVE", this);

        tasks.addFixedDelay(new ImageTask(), 0.04);
        tasks.addFixedDelay(new PoseTask(), 0.04);
        tasks.addFixedDelay(new ControlTask(), 0.04);
        tasks.addFixedDelay(new ClassifyTask(), 0.04);
    }

    public april.sim.Shape getShape()
    {
        return shape;
    }

    private CompoundShape makeShape()
    {
        CompoundShape shape = new CompoundShape();
        if (useCoarseShape) // coarse only
            shape.add(LinAlg.translate(MagicRobot.CENTER_X_OFFSET, 0, 0.5*MagicRobot.COARSE_SIZE[2]),
                      new BoxShape(MagicRobot.COARSE_SIZE));
        else {
            shape.add(
                LinAlg.translate(0, 0, MagicRobot.ORIGIN_Z_OFFSET_GROUND),

                // chassis
                LinAlg.translate(MagicRobot.CHASSIS_MAIN_POS),
                new BoxShape(MagicRobot.CHASSIS_MAIN_SIZE),
                LinAlg.inverse(LinAlg.translate(MagicRobot.CHASSIS_MAIN_POS)),

                LinAlg.translate(MagicRobot.CHASSIS_BATTERY_POS),
                new BoxShape(MagicRobot.CHASSIS_BATTERY_SIZE),
                LinAlg.inverse(LinAlg.translate(MagicRobot.CHASSIS_BATTERY_POS)),

                // front handle
                LinAlg.translate(MagicRobot.HANDLE_FRONT_POS),
                new BoxShape(MagicRobot.HANDLE_CENTER_SIZE),
                LinAlg.inverse(LinAlg.translate(MagicRobot.HANDLE_FRONT_POS)),

                LinAlg.translate(MagicRobot.HANDLE_FRONT_TOP_POS),
                LinAlg.rotateY(Math.PI/6),
                new BoxShape(MagicRobot.HANDLE_OTHER_SIZE),
                LinAlg.rotateY(-Math.PI/6),
                LinAlg.inverse(LinAlg.translate(MagicRobot.HANDLE_FRONT_TOP_POS)),

                LinAlg.translate(MagicRobot.HANDLE_FRONT_BOTTOM_POS),
                LinAlg.rotateY(-Math.PI/6),
                new BoxShape(MagicRobot.HANDLE_OTHER_SIZE),
                LinAlg.rotateY(Math.PI/6),
                LinAlg.inverse(LinAlg.translate(MagicRobot.HANDLE_FRONT_BOTTOM_POS)),

                // rear handle
                LinAlg.translate(MagicRobot.HANDLE_REAR_POS),
                new BoxShape(MagicRobot.HANDLE_CENTER_SIZE),
                LinAlg.inverse(LinAlg.translate(MagicRobot.HANDLE_REAR_POS)),

                LinAlg.translate(MagicRobot.HANDLE_REAR_TOP_POS),
                LinAlg.rotateY(-Math.PI/6),
                new BoxShape(MagicRobot.HANDLE_OTHER_SIZE),
                LinAlg.rotateY(Math.PI/6),
                LinAlg.inverse(LinAlg.translate(MagicRobot.HANDLE_REAR_TOP_POS)),

                LinAlg.translate(MagicRobot.HANDLE_REAR_BOTTOM_POS),
                LinAlg.rotateY(Math.PI/6),
                new BoxShape(MagicRobot.HANDLE_OTHER_SIZE),
                LinAlg.rotateY(-Math.PI/6),
                LinAlg.inverse(LinAlg.translate(MagicRobot.HANDLE_REAR_BOTTOM_POS)),

                // sensor head
                LinAlg.translate(MagicRobot.SENSOR_HEAD_POS),
                new BoxShape(MagicRobot.SENSOR_HEAD_SIZE),
                LinAlg.inverse(LinAlg.translate(MagicRobot.SENSOR_HEAD_POS))
                );
            if (drawSensor) {
                shape.add(

                    );
            }
        }
        return shape;
    }

    public VisObject getVisObject()
    {
        return visObj;
    }

    public double[][] getPose()
    {
        return LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                      drive.poseTruth.pos);
    }

    public void setPose(double T[][])
    {
        drive.poseTruth.orientation = LinAlg.matrixToQuat(T);
        drive.poseTruth.pos = new double[] { T[0][3], T[1][3], 0 };
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }

    private void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("DIFF_DRIVE")) {
            diff_drive_t msg = new diff_drive_t(ins);
            diffdriveCache.put(msg, msg.utime);
        }

        if (channel.startsWith("GAMEPAD")) {
            gamepad_t msg = new gamepad_t(ins);
            gamepadCache.put(msg, msg.utime);
        }
    }

    public void setNoise(boolean noise)
    {
        useNoise = noise;
    }

    class ImageTask implements PeriodicTasks.Task
    {
        double gridmap_range = 10;
        double gridmap_meters_per_pixel = 0.1;

        HashSet<SimObject> ignore = new HashSet<SimObject>();

        public ImageTask()
        {
            ignore.add(SimRobot.this);
        }

        public void run(double dt)
        {
            double radstep = Math.atan2(gridmap_meters_per_pixel, gridmap_range);
            double minDeg = -135;
            double maxDeg = 135;
            double maxRange = 29.9;
            double rad0 = Math.toRadians(minDeg);
            double rad1 = Math.toRadians(maxDeg);

            double T_truth[][] = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                                                        drive.poseTruth.pos),
                                                 LinAlg.translate(0.3, 0, 0.25));

            double T_odom[][] = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseOdom.orientation,
                                                                       drive.poseOdom.pos),
                                                LinAlg.translate(0.3, 0, 0.25));

            double ranges[] = Sensors.laser(sw, ignore, T_truth, (int) ((rad1-rad0)/radstep),
                                            rad0, radstep, maxRange);

            // XXX Config file
            double mean = 0;
            double stddev = 0.01;
            if (useNoise) {
                Random r = new Random();
                for (int i = 0; i < ranges.length; i++) {
                    if (ranges[i] == maxRange)
                        continue;
                    ranges[i] = Math.min(maxRange, ranges[i] + r.nextGaussian()*stddev*ranges[i]);
                }
            }

            laser_t laser = new laser_t();
            laser.utime = TimeUtil.utime();
            laser.nranges = ranges.length;
            laser.ranges = LinAlg.copyFloats(ranges);
            laser.rad0 = (float) rad0;
            laser.radstep = (float) radstep;

            lcm.publish("LASER", laser);
        }
    }

    // XXX Temporary existence inside robot. Longer term, classifications are
    // likely to be spun up outside the robot, but this allows us to quite easily
    // turn noise on and off
    class ClassifyTask implements PeriodicTasks.Task
    {
        TagClassifier tc;

        public ClassifyTask()
        {
            try {
                tc = new TagClassifier(false);
            } catch (IOException ioex) {
                System.err.println("ERR: Could not create TagClassifier");
                ioex.printStackTrace();
            }
        }

        public void run(double dt)
        {
            // Look through all objects in the world and if one is
            // a door and it's within a set distance of us, "classify" it
            double[] xyzrpyBot = LinAlg.matrixToXyzrpy(getPose());

            for(SimObject so : sw.objects) {
                detectDoor(so, xyzrpyBot);
                detectHallway(so, xyzrpyBot);
                detectApriltag(so, xyzrpyBot);
            }
        }

        private void detectDoor(SimObject so, double[] xyzrpyBot)
        {
            if (!(so instanceof SimDoor || so instanceof SimFalseDoor))
                return;
            double sensingThreshold = 1.5;

            double[] xyzrpyDoor = LinAlg.matrixToXyzrpy(so.getPose());
            double dist = LinAlg.distance(xyzrpyBot, xyzrpyDoor, 2);
            if (dist > sensingThreshold)
                return;

            classification_t classies = new classification_t();
            classies.utime = TimeUtil.utime();
            classies.name = "door";

            // Position relative to robot. For now, tossing away orientation data,
            // but may be relevant later.
            double[] relXyzrpy = relativePose(getPose(), xyzrpyDoor);
            classies.xyzrpy = relXyzrpy;

            if (useNoise) {
                // Object detections imperfect. Based on classification confidence
                if (so instanceof SimDoor) {
                    SimDoor door = (SimDoor)so;
                    classies.id = door.id;
                    classies.confidence = sampleConfidence(door.mean, door.stddev);
                } else {
                    SimFalseDoor door = (SimFalseDoor)so;
                    classies.id = door.id;
                    classies.confidence = sampleConfidence(door.mean, door.stddev);
                }
            } else {
                // Object detections perfect. Object MUST be a real door
                if (!(so instanceof SimDoor))
                    return;
                SimDoor door = (SimDoor)so;
                classies.id = door.id;
                classies.confidence = 1.0;
            }

            classification_list_t classy_list = new classification_list_t();
            classy_list.utime = TimeUtil.utime();
            classy_list.num_classifications = 1;
            classy_list.classifications = new classification_t[]{classies};
            lcm.publish("CLASSIFICATIONS", classy_list);
        }

        // Only detect hallways that we can "see". That means that
        // 1) They are not behind the robot
        // 2) The portal opens in an appropriate direction
        private void detectHallway(SimObject so, double[] xyzrpyBot)
        {
            if (!(so instanceof SimHallway))
                return;
            SimHallway hall = (SimHallway)so;

            // Imperfect and probably over generous
            double SENSING_THRESHOLD = 5.0;
            double ORIENTATION_THRESHOLD = Math.toRadians(-5);

            double[] xyzrpyHall = LinAlg.matrixToXyzrpy(so.getPose());
            double[] relXyzrpy = relativePose(getPose(), xyzrpyHall);
            double yawHall = xyzrpyHall[5];
            double yawBot = xyzrpyBot[5];
            double dotp = Math.cos(yawBot)*Math.cos(yawHall) + Math.sin(yawBot)*Math.sin(yawHall);
            double dist = LinAlg.distance(xyzrpyBot, xyzrpyHall, 2);
            // Object must be in range, correctly oriented, and in front of the robot (XXX)
            if (dist > SENSING_THRESHOLD|| dotp < ORIENTATION_THRESHOLD || relXyzrpy[0] < -0.5)
                return;

            classification_t classies = new classification_t();
            classies.utime = TimeUtil.utime();
            classies.name = "hallway";
            classies.xyzrpy = relXyzrpy;
            classies.id = hall.id;

            if (useNoise) {
                classies.confidence = sampleConfidence(hall.mean, hall.stddev);
            } else {
                classies.confidence = 1.0;
            }

            classification_list_t classy_list = new classification_list_t();
            classy_list.utime = TimeUtil.utime();
            classy_list.num_classifications = 1;
            classy_list.classifications = new classification_t[]{classies};
            lcm.publish("CLASSIFICATIONS", classy_list);
        }

        private void detectApriltag(SimObject so, double[] xyzrpyBot)
        {
            if (!(so instanceof SimAprilTag))
                return;
            SimAprilTag tag = (SimAprilTag)so;
            double sensingThreshold = 2.0;

            double[] xyzrpyTag = LinAlg.matrixToXyzrpy(so.getPose());
            double dist = LinAlg.distance(xyzrpyBot, xyzrpyTag, 2);
            if (dist > sensingThreshold)
                return;

            // Position relative to robot. For now, tossing away orientation data,
            // but may be relevant later.
            double[] relXyzrpy = relativePose(getPose(), xyzrpyTag);

            ArrayList<classification_t> classies = tc.classifyTag(tag.getID(), relXyzrpy);

            classification_list_t classy_list = new classification_list_t();
            classy_list.utime = TimeUtil.utime();
            classy_list.num_classifications = classies.size();
            classy_list.classifications = classies.toArray(new classification_t[0]);
            lcm.publish("CLASSIFICATIONS", classy_list);
        }

        private double sampleConfidence(double u, double s)
        {
            return MathUtil.clamp(u + classifierRandom.nextGaussian()*s, 0, 1);
        }

        private double[] relativePose(double[][] A, double[] xyzrpy)
        {
            double[] xyzrpy_A = LinAlg.matrixToXyzrpy(A);
            double[] p = LinAlg.resize(xyzrpy, 3);
            p = LinAlg.transform(LinAlg.inverse(A), p);
            p = LinAlg.resize(p, 6);

            // Relative yaw difference.
            p[5] = MathUtil.mod2pi(xyzrpy[5] - xyzrpy_A[5]);

            return p;
        }
    }

    class PoseTask implements PeriodicTasks.Task
    {
        public PoseTask()
        {
        }

        public void run(double dt)
        {
            pose_t pose;
            if (useNoise) {
                pose = drive.poseOdom;
            } else {
                pose = drive.poseTruth;
            }
            lcm.publish("POSE", pose);
        }
    }

    class ControlTask implements PeriodicTasks.Task
    {
        Params params = Params.makeParams();

        public void run(double dt) {
            double[] mcmd = new double[2];

            double center_xyz[] = LinAlg.add(drive.poseOdom.pos, LinAlg.quatRotate(drive.poseOdom.orientation, drive.centerOfRotation));

            double q[] = drive.poseOdom.orientation;

            diff_drive_t dd = diffdriveCache.get();
            if (dd != null) {
                mcmd = new double[] { dd.left, dd.right };
            }
            // Gamepad override
            gamepad_t gp = gamepadCache.get();
            if (gp != null) {

                final int RIGHT_VERT_AXIS = 5;//3;
                final int RIGHT_HORZ_AXIS = 4;//2;

                double speed = -gp.axes[RIGHT_VERT_AXIS];
                if ((gp.buttons & (16 | 32)) == (16 | 32))  // if holding both buttons go faster
                    speed *= 4;

                double turn = gp.axes[RIGHT_HORZ_AXIS];

                if (gp.buttons != 0) {
                    mcmd = new double[] { speed + turn, speed - turn };
                }
            }

            drive.motorCommands = mcmd;
        }
    }

    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
        this.robotID = ins.readInt();

        double Ttruth[][] = LinAlg.xyzrpyToMatrix(ins.readDoubles());
        double Todom[][] = LinAlg.xyzrpyToMatrix(ins.readDoubles());

        synchronized(drive) {
            drive.poseTruth.orientation = LinAlg.matrixToQuat(Ttruth);
            drive.poseTruth.pos = new double[] { Ttruth[0][3], Ttruth[1][3], Ttruth[2][3] };

            drive.poseOdom.orientation = LinAlg.matrixToQuat(Todom);
            drive.poseOdom.pos = new double[] { Todom[0][3], Todom[1][3], Todom[2][3] };
        }
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
        double Ttruth[][] = LinAlg.quatPosToMatrix(drive.poseTruth.orientation, drive.poseTruth.pos);
        double Todom[][] = LinAlg.quatPosToMatrix(drive.poseOdom.orientation, drive.poseOdom.pos);

        outs.writeComment("Robot ID");
        outs.writeInt(robotID);
        outs.writeComment("XYZRPY Truth");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(Ttruth));
        outs.writeComment("XYZRPY Odom");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(Todom));
    }

    public synchronized void setRunning(boolean b)
    {
        drive.setRunning(b);
        tasks.setRunning(b);
    }
}
