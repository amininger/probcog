package probcog.slam;

import java.util.*;

import april.jmat.*;

import magic2.lcmtypes.*;

/** Convert odometry messages to appropriate pose messages. */
public class Odometry
{
    static final double D_BIAS = 0.998; // [0,1]
    static final double TICKS_2_METERS = 3.99072e-4;
    //static final double GYROS_2_RADS = 9.14323799542e-11;
    static final double GYROS_2_RADS = 2.6631588e-10;   // 1/2^16/1000*(2Pi/360)

    odom_imu_t lastOdom = null;
    double yawBias = 0.0;
    int biasSamples = 0;
    double yawGyroAcc = 0.0;
    double x = 0.0;
    double y = 0.0;

    public Odometry()
    {
    }

    /** Given an input odometry message, recover a pose */
    public pose_t getPose(odom_imu_t odom)
    {
        pose_t pose = new pose_t();
        pose.orientation = new double[] {1,0,0,0};

        // Update
        if (lastOdom != null) {
            double dl = odom.left_encoder - lastOdom.left_encoder;
            double dr = odom.right_encoder - lastOdom.right_encoder;
            int yawDiff= odom.gyro[2] - lastOdom.gyro[2];

            // Handle bias updates. If our wheels weren't moving, assume we
            // were stationary and try to calibrate bias.
            if (dl == 0 && dr == 0) {
                //yawBias = (yawBias*biasSamples + yawDiff)/(biasSamples+1);
                yawBias = D_BIAS*yawBias + (1-D_BIAS)*yawDiff;
                biasSamples++;
            }

            yawGyroAcc += yawDiff - yawBias;
            double yaw = MathUtil.mod2pi(yawGyroAcc*GYROS_2_RADS);
            double dist = 0.5 * (dl + dr) * TICKS_2_METERS;
            x += dist * Math.cos(yaw);
            y += dist * Math.sin(yaw);

            pose.utime = odom.utime;
            pose.pos = new double[] {x, y, 0};
            pose.orientation = LinAlg.matrixToQuat(LinAlg.rotateZ(yaw));
        }

        lastOdom = odom;
        return pose;
    }
}
