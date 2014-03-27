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

import probcog.lcmtypes.*;
import probcog.util.*;
import probcog.vis.*;
import probcog.robot.control.*;
import probcog.sensor.SimKinectSensor;

public class SimRobot implements LCMSubscriber, SimObject
{
    SimWorld sw;
    DifferentialDrive drive;

    boolean useCoarseShape;
    boolean drawSensor;
    int robotID;

    LCM lcm = LCM.getSingleton();

    CompoundShape shape;
    VisObject visObj;
    SimKinectSensor kinect;

    ExpiringMessageCache<gamepad_t> gamepadCache = new ExpiringMessageCache<gamepad_t>(0.25);

    PeriodicTasks tasks = new PeriodicTasks(2);

    // static variables
    static Random r = new Random();
    static Model4 model4 = new Model4();

    public SimRobot(SimWorld sw, int robotID)
    {
        this.sw = sw;
        useCoarseShape = sw.config.getBoolean("simulator.sim_magic_robot.use_coarse_shape", true);
        drawSensor = sw.config.getBoolean("simulator.sim_magic_robot.draw_sensor", false);
        this.robotID = robotID;

        shape = makeShape();

        if (drawSensor) {
            visObj = new VisChain(model4, Model4.Model4Head(Util.getConfig(), 0, 0, 0));
        } else
            visObj = model4;

        drive = new DifferentialDrive(sw, this, new double[3]);
        drive.centerOfRotation = new double[] { 0.13, 0, 0 };

        lcm.subscribe("GAMEPAD_"+robotID, this);
        lcm.subscribe("SOAR_COMMAND", this);

        tasks.addFixedDelay(new ImageTask(), 2.0);
        tasks.addFixedDelay(new ControlTask(), 0.04);
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
        // XXX - New code goes in here
        if (channel.equals("SOAR_COMMAND")) {


        }

        if (channel.startsWith("GAMEPAD")) {
            gamepad_t msg = new gamepad_t(ins);
            gamepadCache.put(msg, msg.utime);
        }
    }

    class ImageTask implements PeriodicTasks.Task
    {
        int frameCount = 0;

        public void run(double dt)
        {
            frameCount++;

            boolean sendKinect = true;
            if (sendKinect) {
                // Where is the robot in the world?
                double R2G[][] = LinAlg.quatPosToMatrix(drive.poseTruth.orientation, drive.poseTruth.pos);
                // Where is the kinect with respect to the robot origin (rear axle)?
                double K2R[][] = LinAlg.translate(0.21 + 0.055, 0, 0.68); // XXX - Need to get correct

                // Where is the kinect in the world?
                double K2G[][] = LinAlg.matrixAB(R2G, K2R);

                double q[] = LinAlg.matrixToQuat(K2G);
                double eye[] = LinAlg.matrixAB(K2G, new double[] {0,0,0,1});
                double lookAt[] = LinAlg.matrixAB(K2G, new double[] {0,0,0,1});
                double up[] = LinAlg.quatRotate(q, new double[] { 0, 0, 1 });

                // construct the appropriate visworld
                VisWorld vw = new VisWorld();
                synchronized(sw) {
                    VisWorld.Buffer vb = vw.getBuffer("world");
                    for (SimObject obj : sw.objects) {
                        vb.addBack(new VisChain(obj.getPose(), obj.getVisObject()));
                    }
                    vb.addBack(new VzGrid());
                    vb.swap();
                }

                kinect = new SimKinectSensor(sw, eye, lookAt, up);
                ArrayList<double[]> xyzrpy = kinect.getAllXYZRGB();
                // publish a frame here ?
                laser_t las = new laser_t(ins);
                // las
            }
        }
    }

    class ControlTask implements PeriodicTasks.Task
    {
        Params params = Params.makeParams();

        public void run(double dt) {
            double[] mcmd = new double[2];

            double center_xyz[] = LinAlg.add(drive.poseOdom.pos, LinAlg.quatRotate(drive.poseOdom.orientation, drive.centerOfRotation));

            double q[] = drive.poseOdom.orientation;

            diff_drive_t dd = PathControl.getDiffDrive(center_xyz,
                                                       q,
                                                       wpp_path,
                                                       params,
                                                       1.0);
            mcmd = new double[] { dd.left, dd.right };

            // Gamepad override
            gamepad_t gp = gamepadCache.get();
            if (gp != null) {

                final int RIGHT_VERT_AXIS = 3;
                final int RIGHT_HORZ_AXIS = 2;

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
