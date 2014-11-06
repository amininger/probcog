package probcog.slam;

import java.util.*;

import april.jmat.*;
import april.sim.*; // Motor
import april.util.*;

import magic2.lcmtypes.*; // XXX

/** A simulated robot designed to drive around a TagMap.
 *
 *  Represented as a point robot, so it's important to make
 *  sure all navigation is on the appropriately modeled
 *  configuration space for the real robot being simulated.
 */
public class TagRobot
{
    Random r = new Random();

    public Motor leftMotor = new Motor();
    public Motor rightMotor = new Motor();
    public double voltageScale = 24.0;

    // XXX Init from config?
    // XXX Before, we used Util.getConfig()...Ick.
    static final double DEFAULT_DT = 0.2;

    // True pose of the robot
    public pose_t poseTruth = new pose_t();

    // A noise-corrupted version of the above
    public pose_t poseOdom = new pose_t();

    // Robot params
    public double wheelDiameter = 0.25; // diamter of the wheels [m]
    public double baseline = 0.46;      // distance between L and R wheels [m]

    // Noise params. Tweak for new robots XXX
    double translation_noise = 0.1;
    double rotation_noise = 0.05;

    public TagRobot(double[] xyt)
    {
        poseTruth.pos = new double[] {xyt[0], xyt[1], 0};
        poseTruth.orientation = LinAlg.rollPitchYawToQuat(new double[] {0, 0, xyt[2]});
        poseOdom = poseTruth.copy();

        // Motor setup
        double K_t = 0.7914*2;      // Torque constant in [Nm/A]
        leftMotor.torque_constant = K_t;
        rightMotor.torque_constant = K_t;
        double K_emf = 1.406;       // emf constant [V/(rad/s)]
        leftMotor.emf_constant = K_emf;
        rightMotor.emf_constant = K_emf;
        double K_wr = 5.5;          // XXX Old winding resistance [ohms]
        leftMotor.winding_resistance = K_wr;
        rightMotor.winding_resistance = K_wr;
        double K_inertia = 0.5;     // Old inertia [kg m^2]
        leftMotor.inertia = K_inertia;
        rightMotor.inertia = K_inertia;
        double K_drag = 1.0;        // Old drag [Nm / (rad/s)], always >= 0
        leftMotor.drag_constant = K_drag;
        rightMotor.drag_constant = K_drag;
    }

    public void update(diff_drive_t dd, GridMap gm)
    {
        update(dd, gm, DEFAULT_DT);
    }

    public void update(diff_drive_t dd, GridMap gm, double dt)
    {
        // Get motor command updates
        if (dd.left_enabled)
            leftMotor.setVoltage(dd.left*voltageScale);
        else
            leftMotor.setVoltage(0);

        if (dd.right_enabled)
            rightMotor.setVoltage(dd.right*voltageScale);
        else
            rightMotor.setVoltage(0);

        leftMotor.update(dt);
        rightMotor.update(dt);

        // Apply the motor updates.
        double left_rad_per_sec = leftMotor.getRadPerSec();
        double right_rad_per_sec = rightMotor.getRadPerSec();

        double dl = dt * left_rad_per_sec * wheelDiameter;
        double dr = dt * right_rad_per_sec * wheelDiameter;

        double dl_truth = (dl+dr) / 2;
        double dtheta_truth = (dr-dl) / baseline;

        // Pollute the truth with noise
        double r1 = MathUtil.clamp(r.nextGaussian(), -3.0, 3.0);
        double r2 = MathUtil.clamp(r.nextGaussian(), -3.0, 3.0);

        double dl_odom = dl_truth + translation_noise*r1*Math.abs(dl_truth);
        double dtheta_odom = dtheta_truth + rotation_noise*r2*Math.abs(dtheta_truth);

        // Apply updates to poses
        double[] dpos_truth = LinAlg.quatRotate(poseTruth.orientation, new double[] {dl_truth, 0, 0});
        double[] dquat_truth = LinAlg.rollPitchYawToQuat(new double[] {0, 0, dtheta_truth});

        double[] dpos_odom = LinAlg.quatRotate(poseOdom.orientation, new double[] {dl_odom, 0, 0});
        double[] dquat_odom = LinAlg.rollPitchYawToQuat(new double[] {0, 0, dtheta_odom});

        double[] newpos_truth = LinAlg.add(poseTruth.pos, dpos_truth);
        double[] neworient_truth = LinAlg.quatMultiply(poseTruth.orientation, dquat_truth);

        double[] newpos_odom = LinAlg.add(poseOdom.pos, dpos_odom);
        double[] neworient_odom = LinAlg.quatMultiply(poseOdom.orientation, dquat_odom);

        poseTruth.utime += dt*1000000;
        poseOdom.utime = poseTruth.utime;

        // Verify that we didn't collide with something.
        if (gm.evaluatePath(poseTruth.pos, newpos_truth, true) < 0) {
            // Collision. Don't accept this movement
            return;
        }

        poseTruth.pos = newpos_truth;
        poseTruth.orientation = neworient_truth;

        poseOdom.pos = newpos_odom;
        poseOdom.orientation = neworient_odom;
    }


}
