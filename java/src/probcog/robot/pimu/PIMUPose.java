package probcog.robot.pimu;

import lcm.lcm.*;
import april.lcmtypes.*;
import april.jmat.*;
import april.util.*;
import april.config.*;

import java.io.*;
import java.util.*;

import probcog.lcmtypes.*;

// import magic.util.*;

public class PIMUPose implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();
    pose_t pose;
    double lasts;

    double distTraveled = 0;
    double lastz = 0;   // The last z-value from pose transform
    double repz = 0;   // The last z-value sent out as pose

    //double velscale = 0.000325; // full-resolution encoders
    double velscale = 0.00070652; // half-resolution encoders

    PIMUFilter filter = new PIMUFilter();
    GetOpt gopt;

    int motorFeedbackCount, pimuCount, txCount;

    Config config = RobotUtil.getConfig();

    public PIMUPose(GetOpt gopt)
    {
        this.gopt = gopt;

        pose = new pose_t();
        pose.orientation = LinAlg.rollPitchYawToQuat(new double[3]);

        lcm.subscribe("PIMU", this);
        lcm.subscribe("MOTOR_FEEDBACK", this);

        new PublishThread().start();
        new ReportThread().start();
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("PIMU")) {
                pimu_t d = new pimu_t(ins);
                handleIMU(d);
                pimuCount++;
            }

            if (channel.equals("MOTOR_FEEDBACK")) {
                motor_feedback_t d = new motor_feedback_t(ins);
                handleMotorFeedback(d);
                motorFeedbackCount++;
            }

        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }

    void handleMotorFeedback(motor_feedback_t d)
    {
        double s = velscale*(d.encoders[1] + d.encoders[2]) / 2.0;

        double ds = s - lasts;
        distTraveled += ds;

        if (Math.abs(ds) < 1) {
            // robot moves down the X axis in its own coordinate frame.
            double m[] = new double[] { ds, 0, 0 };

            synchronized(pose) {
                pose.pos = LinAlg.add(pose.pos, LinAlg.quatRotate(pose.orientation, m));
                pose.utime = d.utime;
            }
        }

        lasts = s;
    }

    void handleIMU(pimu_t d)
    {
        filter.update(d);

        synchronized(pose) {
            pose.orientation = filter.getQuaternion();
            pose.rotation_rate = filter.getAngularRates();
            // use only encoder utimes for better monitonicity
//          pose.utime = d.utime;
        }
    }

    public class PublishThread extends Thread
    {
        double zdecay = 0.02;   // z decay rate towards zero in meters per meter traveled

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000 / gopt.getInt("hz"));
                txCount++;

                pose_t poseAdj;

                synchronized(pose) {
                    poseAdj = pose.copy();
                }

                double dx = config.requireDouble("robot.geometry.circles_x");
                double dpos[] = LinAlg.quatRotate(pose.orientation, new double[] { -dx, 0, 0});
                poseAdj.pos = LinAlg.add(poseAdj.pos, dpos);

                // Correct for z drift away from zero as robot moves
                double dz = poseAdj.pos[2] - lastz;     // Z-drift between updates
                lastz = poseAdj.pos[2];

                repz += dz;        // Add drift to our last reported z value
                double dir = repz > 0 ? -1.0 : 1.0;
                repz += (zdecay*distTraveled*dir);  // drag the reported z back towards zero as we move
                double min = dir > 0 ? -Double.MAX_VALUE : 0;
                double max = dir > 0 ? 0 : Double.MAX_VALUE;
                repz = MathUtil.clamp(repz, min, max);
                poseAdj.pos[2] = repz;

                distTraveled = 0;   // Reset distance traveled since last update

                // XXX SPECIAL FLATWORLD ASSUMPTION:
                if (config.requireBoolean("obstacle.use_flat_world")) {
                    poseAdj.pos[2] = 0;
                }

                lcm.publish("POSE", poseAdj);
            }
        }
    }

    public class ReportThread extends Thread
    {
        public void run()
        {
            while (true) {
                long utime0 = TimeUtil.utime();
                TimeUtil.sleep(1000);
                long utime1 = TimeUtil.utime();
                double dt = (utime1 - utime0) / 1000000.0;
                System.out.printf("%15.3f : PIMUPose encoder=%.1f Hz, pimu=%.1f Hz, tx=%.1f Hz\n", utime1 / 1000000.0,
                                  motorFeedbackCount / dt, pimuCount / dt, txCount / dt);
                pimuCount = 0;
                txCount = 0;
                System.out.println("Dist traveled: " + distTraveled);
            }
        }
    }

    public static void main(String args[])
    {
        GetOpt gopt = new GetOpt();
        gopt.addInt('\0', "hz", 100, "Publish rate (Hz)");
        gopt.addBoolean('h', "help", false, "Show this help");

        if (!gopt.parse(args)) {
            gopt.doHelp();
            System.exit(-1);
        }

        new PIMUPose(gopt);

        while (true) {
            TimeUtil.sleep(100);
        }
    }
}
