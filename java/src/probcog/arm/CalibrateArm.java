package probcog.arm;

import java.awt.*;
import java.io.*;
import javax.swing.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.util.*;
import april.vis.*;

import probcog.gui.*;
import probcog.lcmtypes.*;

/** Move the arm to a preset number of positions, measuring
 *  the actual assumed position after each command is finished.
 *  Use these to compute a mapping from desired position to
 *  the position that must be commanded to best achieve that
 *  goal.
 */
public class CalibrateArm implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    ArmStatus arm;

    static VisWorld vw = new VisWorld();
    static VisLayer vl = new VisLayer(vw);
    static VisCanvas vc = new VisCanvas(vl);

    // Action tracking
    Object actionLock = new Object();
    robot_action_t lastAction = null;

    ArmCalibration calibration;

    // XXX Ew
    public CalibrateArm(String filename)
    {
        calibration = new ArmCalibration(filename);
        calibration.validate();
    }


    public CalibrateArm(Config config) throws IOException
    {
        arm = new ArmStatus(config);

        lcm.subscribe("ROBOT_ACTION", this);

        calibrationDance();
        calibration.validate();
        //validateCalibration();
    }

    /** Handle incoming LCM messages */
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("ERR: Could not handle LCM channel - "+channel);
            ex.printStackTrace();
        }
    }

    /** Responds to messages such as arm statuses and robot commands. */
    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("ROBOT_ACTION")) {
            synchronized (actionLock) {
                lastAction = new robot_action_t(ins);
            }
        }
    }

    private void calibrationDance()
    {
        // Parameters
        double minR = 0.15;
        double maxR = 0.30;
        double stepR = 0.049;

        double minT = -Math.PI+0.0001;
        double maxT = Math.PI;
        double stepT = Math.toRadians(29.9);

        double minH = 0.05;
        double maxH = 0.15;
        double stepH = 0.049;

        // Exhaustively enumerate all positions defined by the parameters
        // and compute mappings of requested positions to actual
        byte cmd_id = 0;
        ArrayList<double[]> requested = new ArrayList<double[]>();
        ArrayList<double[]> actual = new ArrayList<double[]>();
        for (double t = minT; t <= maxT; t += stepT) {
            for (double r = minR; r <= maxR; r += stepR) {
                for (double h = minH; h <= maxH; h += stepH) {
                    double[] xyz_r = new double[3];
                    xyz_r[0] = r*Math.cos(t);
                    xyz_r[1] = r*Math.sin(t);
                    xyz_r[2] = h;

                    bolt_arm_command_t cmd = new bolt_arm_command_t();
                    cmd.utime = TimeUtil.utime();
                    cmd.cmd_id = cmd_id++;
                    cmd.action = "EXACT";
                    cmd.xyz = xyz_r;
                    cmd.wrist = 0;
                    cmd.obj_id = -1;

                    requested.add(xyz_r);
                    lcm.publish("BOLT_ARM_COMMAND", cmd);

                    // Wait for the arm to finish executing the command. This
                    // will be signalled by a return to WAIT mode in the robot
                    // action message
                    int pauseMilli = 50;
                    long waitTime = 10*1000000;
                    long lastTime = TimeUtil.utime();

                    boolean sawExact = false;
                    while (true) {
                        arm.render(vw);
                        long currTime = TimeUtil.utime();
                        waitTime -= (currTime - lastTime);
                        lastTime = currTime;
                        if (waitTime < 0) {
                            System.err.println("ERR: Timed out waiting for robot to complete action");
                            System.exit(1);
                        }
                        synchronized (actionLock) {
                            if (lastAction == null) {
                                TimeUtil.sleep(pauseMilli);
                                continue;
                            }
                            if (lastAction.action.equals("EXACT")) {
                                sawExact = true;
                            }
                            if (sawExact && lastAction.action.equals("WAIT")) {
                                break;
                            }
                        }
                    }

                    // Get arm position. We've finished our action
                    double[] xyz_c = LinAlg.resize(arm.getGripperXYZRPY(), 3);
                    actual.add(xyz_c);

                    // XXX DEBUG
                    //System.out.printf("[%2.4f, %2.4f, %2.4f] --> [%2.4f, %2.4f, %2.4f]\n",
                    //                  xyz_r[0], xyz_r[1], xyz_r[2],
                    //                  xyz_c[0], xyz_c[1], xyz_c[2]);
                }
            }
        }

        // At this point we do something with the calibration
        calibration = new ArmCalibration(requested, actual);
        calibration.save("arm_calibration.cal");
    }

    private void validateCalibration()
    {
        // XXX This is, at the moment, only a pseudo validation. Not a real
        // evaluation of the system so much as proof that it's doing the
        // right thing.
        System.out.println("Validating calibration...");
        for (double x = -.3; x <= .3; x += 0.05) {
            for (double y = .1; y <= .3; y+= 0.5) {
                for (double z = 0.05; z <= 0.15; z += 0.25) {
                    double[] xyz_r = new double[] {x,y,z};
                    double[] xyz_c = calibration.map(xyz_r);

                    // XXX DEBUG
                    System.out.printf("[%2.4f, %2.4f, %2.4f] --> [%2.4f, %2.4f, %2.4f]\n",
                                      xyz_r[0], xyz_r[1], xyz_r[2],
                                      xyz_c[0], xyz_c[1], xyz_c[2]);
                }
            }
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h',"help",false,"Show this help screen");
        opts.addString('c',"config",null,"Config file");
        opts.addString('w',"world",null,"Sim world");
        opts.addString('a',"arm",null,"Arm calibration");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help") ||
            opts.getString("config") == null)
        {
            opts.doHelp();
            System.exit(0);
        }

        Config config = null;
        try {
            config = new ConfigFile(opts.getString("config"));
        } catch (IOException ioex) {
            System.err.println("ERR: Could not open config file");
            ioex.printStackTrace();
            System.exit(1);
        }

        if (opts.getString("arm") != null) {
            new CalibrateArm(opts.getString("arm"));
            System.exit(0);
        }

        // Spin up appropriate widgets and run calibration
        try {
            System.out.println("ATTN: Don't forget to set the appropriate PID values");

            ArmController ac = new ArmController(config);
            if (opts.getString("world") != null) {
                JFrame jf = new JFrame("Calibration Dance Debugger");
                jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
                jf.setLayout(new BorderLayout());
                jf.setSize(800, 600);

                jf.add(vc, BorderLayout.CENTER);

                jf.setVisible(true);
                ProbCogSimulator simulator = new ProbCogSimulator(opts, vw, vl, vc);
                SimArm simArm = new SimArm(config, simulator.getWorld());
            } else {
                ArmDriver ad = new ArmDriver(config);
                (new Thread(ad)).start();
            }
            // Command interpretation handled by us.
            CalibrateArm calibrator = new CalibrateArm(config);
        } catch (IOException ioex) {
            System.err.println("ERR: Could not read config file or similar");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}
