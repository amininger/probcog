package probcog.sim;

import java.util.*;

import april.sim.*;
import april.util.*;
import april.jmat.*;
import april.lcmtypes.*;

/** A class that acts as a fixed-speed update DifferentialDrive. */
public class FastDrive
{
    public Motor leftMotor = new Motor();
    public Motor rightMotor = new Motor();
    public double voltageScale = 12.0;

    // True pose of the robot's rear axle (see centerOfRotation)
    public pose_t poseTruth = new pose_t();

    // A noise-currupted version of poseTruth
    public pose_t poseOdom = new pose_t();

    // The position of the robot's center of rotation w.r.t. the rear axle.
    public double[] centerOfRotation = new double[] { 0, 0, 0 };

    // [left, right] where each motor is [-1,1]
    public double[] motorCommands = new double[2];

    // XXX Move to config?
    public double wheelDiameter = 0.25; // diameter of wheels [m]
    public double baseline = 0.35;      // distance between left and right wheels [m]

    // Noise parameters
    public double translation_noise = 0.1;
    public double rotation_noise = 0.05;

    SimWorld sw;
    SimObject simobj;   // Object representing the robot
    Random r = new Random();

    /** Ignore: a set of objects that will not be used for collision detection.
     *  Typically, this includes the robot itself.
     **/
    public FastDrive(SimWorld vw, SimObject simobj, double[] init_xyt)
    {
        this.sw = sw;
        this.simobj = simobj;

        poseTruth.utime = TimeUtil.utime(); // Not really important
        poseTruth.pos = new double[] { init_xyt[0], init_xyt[1], 0 };
        poseTruth.orientation = LinAlg.rollPitchYawToQuat(new double[] { 0, 0, init_xyt[2] });

        poseOdom = poseTruth.copy();
        // Historically, this is a copy of the poseTruth reset to 0s...we'll
        // allow it to be perfect

        // No fixed delay tasks, here, since we update on request!
    }

    final public static double DT = 1.0 / 50;
    public void update()
    {
        synchronized (sw) {
            synchronized (this) {
                leftMotor.setVoltage(motorCommands[0]*voltageScale);
                rightMotor.setVoltage(motorCommands[1]*voltageScale);

                leftMotor.update(DT);
                rightMotor.update(DT);

                // Temporarily make poseTruth point to center of rotation, which
                // we'll undo at the end.
                double[] pos_truth = LinAlg.add(poseTruth.pos, LinAlg.quatRotate(poseTruth.orientation, centerOfRotation));
                double[] pos_odom = LinAlg.add(poseOdom.pos, LinAlg.quatRotate(poseOdom.orientation, centerOfRotation));

                double left_rad_per_sec = leftMotor.getRadPerSec();
                double right_rad_per_sec = rightMotor.getRadPerSec();

                double dleft = DT * left_rad_per_sec * wheelDiameter;
                double dright = DT * right_rad_per_sec * wheelDiameter;

                double dl_truth = (dleft + dright) / 2;
                double dtheta_truth = (dright - dleft) / baseline;

                double r1 = MathUtil.clamp(r.nextGaussian(), -3.0, 3.0);
                double r2 = MathUtil.clamp(r.nextGaussian(), -3.0, 3.0);

                double dl_odom = dl_truth + translation_noise*r1*Math.abs(dl_truth);
                double dtheta_odom = dtheta_truth + rotation_noise*r2*Math.abs(dtheta_truth);

                // Updates
                double[] dpos_truth = LinAlg.quatRotate(poseTruth.orientation, new double[] {dl_truth, 0, 0});
                double[] dquat_truth = LinAlg.rollPitchYawToQuat(new double[] {0, 0, dtheta_truth});

                double[] dpos_odom = LinAlg.quatRotate(poseOdom.orientation, new double[] {dl_odom, 0, 0,});
                double[] dquat_odom = LinAlg.rollPitchYawToQuat(new double[] {0, 0, dtheta_odom});

                double[] newpos_truth = LinAlg.add(pos_truth, dpos_truth);
                double[] neworient_truth = LinAlg.quatMultiply(poseTruth.orientation, dquat_truth);

                double[] newpos_odom = LinAlg.add(pos_odom, dpos_odom);
                double[] neworient_odom = LinAlg.quatMultiply(poseOdom.orientation, dquat_odom);

                // Back to rear axle
                newpos_truth = LinAlg.add(newpos_truth, LinAlg.quatRotate(neworient_truth, LinAlg.scale(centerOfRotation, -1)));
                newpos_odom = LinAlg.add(newpos_odom, LinAlg.quatRotate(neworient_odom, LinAlg.scale(centerOfRotation, -1)));

                // Only accept movements that don't run into things
                boolean okay = true;
                for (SimObject so: sw.objects) {
                    if (so == simobj)
                        continue;
                    if (Collisions.collision(so.getShape(), so.getPose(),
                                             simobj.getShape(), LinAlg.quatPosToMatrix(neworient_truth, newpos_truth))) {
                        okay = false;
                        break;
                    }
                }

                if (okay) {
                    poseTruth.pos = newpos_truth;
                    poseTruth.orientation = neworient_truth;

                    poseOdom.pos = newpos_odom;
                    poseOdom.orientation = neworient_odom;
                }

                poseTruth.utime += DT*1000000;  // Fixed time update
                poseOdom.utime = poseTruth.utime;
            }
        }
    }
}
