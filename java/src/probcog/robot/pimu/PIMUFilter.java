package probcog.robot.pimu;

import java.util.*;
import java.io.*;
import april.util.*;
import april.jmat.*;

import probcog.lcmtypes.*;

/** Estimate orientation given PIMU data **/
public class PIMUFilter
{
    ///////////////////////////////////
    // Core state
    LinkedList<pimu_t> queue = new LinkedList<pimu_t>();
    pimu_t lastData;

    double T[][] = LinAlg.identity(4);
    double angularRates[] = new double[3];

    public boolean verbose = true;

    ///////////////////////////////////
    // attitude correction
    public boolean enableAccelerometers = true;

    double accel_acc[] = new double[3];
    double accel_acc_t;
    double last_up_uncertainty;

    public double maxv = 2.0; // maximum speed of vehicle (m/s)
    public double maxintperiod = 8; // maximum period of time to integrate gravity vector.

    public static final double G = 9.80665; // m/s^2 of earth gravity.

    ///////////////////////////////////
    // Gyro calibration
    public boolean enableGyroCal = true;

    double integrator_scale[] = new double[] { 1, 1, 1, 1, 1, 1, 1, 1 };
    double integrator_offset[] = new double[8]; // in degrees per second
    double integrator_corrected[] = new double[8];

    static final double MIN_COVARIANCE = 1.8;
    double integrator_offset_cov[] = new double[8];

    int useYawGyros = 0; // 0 = both. 1 = first one only. 2 = second one only.

    public PIMUFilter()
    {
    }

    public synchronized PIMUFilter copy()
    {
        PIMUFilter pf = new PIMUFilter();
        pf.integrator_offset = LinAlg.copy(integrator_offset);
        pf.integrator_offset_cov = LinAlg.copy(integrator_offset_cov);
        pf.accel_acc = LinAlg.copy(accel_acc);
        pf.accel_acc_t = accel_acc_t;

        for (pimu_t d : queue)
            pf.queue.add(d);

        pf.lastData = lastData;
        pf.T = LinAlg.copy(T);

        pf.maxv = maxv;
        pf.maxintperiod = maxintperiod;

        pf.integrator_scale = LinAlg.copy(integrator_scale);
        pf.verbose = verbose;
        pf.enableGyroCal = enableGyroCal;
        pf.enableAccelerometers = enableAccelerometers;
        pf.useYawGyros = useYawGyros;

        return pf;
    }

    public synchronized void setQuaternion(double q[])
    {
        T = LinAlg.quatToMatrix(q);
    }

    public synchronized double[] getQuaternion()
    {
        return LinAlg.matrixToQuat(T);
    }

    public synchronized double[] getAngularRates()
    {
        return LinAlg.copy(angularRates);
    }

    public synchronized double[][] getMatrix()
    {
        return LinAlg.copy(T);
    }

    /** Return the estimate of the -gravity direction, in IMU
     * coordinates as measured by the accelerometers. **/
    public synchronized double[] getAccelUp()
    {
        if (accel_acc_t == 0)
            return new double[] {0, 0, 1};
        return LinAlg.normalize(accel_acc);
    }

    /** Returns radians **/
    public synchronized double getUpUncertainty()
    {
        return last_up_uncertainty;
    }

    static boolean once;

    public synchronized void update(pimu_t d)
    {
        // always make sure lastData is non-null
        if (lastData == null)
            lastData = d;

        // reset if time jumps too much. (100ms)
        if (Math.abs(d.utime_pimu - lastData.utime_pimu) > 100000) {
            lastData = d;
            LinAlg.clear(accel_acc);
            accel_acc_t = 0;
            return;
        }

        if (enableGyroCal) {
            // test for zero-velocity.
            queue.add(d);

            double mindt = 0.5;
            int minsamples = 30;

            pimu_t d0 = queue.getFirst();
            pimu_t d1 = queue.getLast();
            double dt = (d1.utime_pimu - d0.utime_pimu) / 1.0E6;

            // time warp? reset.
            if (Math.abs(dt) > mindt * 10) {
                queue.clear();
                dt = 0;
            }

            if (dt > mindt) {
                if (queue.size() > minsamples) {

                    for (int i = 0; i < 8; i++) {
                        ArrayList<double[]> xys = new ArrayList<double[]>();
                        for (pimu_t dd : queue)
                            xys.add(new double[] { dd.utime_pimu / 1.0E6, dd.integrator[i] / 1.0E6 });

                        double fit[] = LinAlg.fitLine(xys);
                        fit[2] /= queue.size();

                        double cov = Math.pow(1.0E6*fit[2], 2);

                        if (integrator_offset[i] == 0) {
                            integrator_offset[i] = fit[0];
                            integrator_offset_cov[i] = cov;
                        } else {
                            // EKF update of slope
                            integrator_offset[i] = (cov*integrator_offset[i] + integrator_offset_cov[i]*fit[0]) /
                                (cov + integrator_offset_cov[i]);

                            integrator_offset_cov[i] = Math.max(MIN_COVARIANCE, 1.0 / (1.0 / cov + 1.0 / integrator_offset_cov[i]));
                        }
                    }

                    if (verbose) {
                        System.out.printf("%10d : ", d.utime_pimu);
                        int axes[] = new int[] { 0, 4, 3, 7 };
                        for (int i = 0; i < axes.length; i++) {
                            System.out.printf("%9.2f (%6f) ", integrator_offset[axes[i]], integrator_offset_cov[axes[i]]);
                        }
                        System.out.printf("\n");
                    }
                } else {
                    System.out.println("WRN: Not enough samples to fit scale factor");
                }

                queue.clear();
            }
        }

        double dT[][] = null;

        if (true) {
            double ws[] = new double[8]; // rotational rates. deg/s
            double wdt = (d.utime_pimu - lastData.utime_pimu) / 1.0E6;  // seconds

            for (int i = 0; i < 8; i++) {
                double rot_deg = (d.integrator[i] - lastData.integrator[i]) / 1.0E6;
                ws[i] = integrator_scale[i]*(rot_deg - integrator_offset[i]*wdt);
                integrator_corrected[i] += ws[i]*1.0E6;
            }

            double rpy[] = null;
            if (useYawGyros == 1)
                rpy = new double[] { -ws[3], -ws[7], ws[0] };
            else if (useYawGyros == 2)
                rpy = new double[] { -ws[3], -ws[7], ws[4] };
            else
                rpy = new double[] { -ws[3], -ws[7], (ws[0] + ws[4]) / 2.0 };

            rpy = LinAlg.scale(rpy, Math.PI / 180);
            angularRates = LinAlg.scale(rpy, Math.PI/180*1.0/wdt);

            dT = LinAlg.rollPitchYawToMatrix(rpy);

            T = LinAlg.matrixAB(T, dT);
        }

        double accelscale = 1.0 / 16384; // to g's.

        double accel[] = new double[] { d.accel[0] * accelscale,
                                        d.accel[1] * accelscale,
                                        d.accel[2] * accelscale };


        if (enableAccelerometers) {
            /*
               Let us assume that we can bound the average
               acceleration of the vehicle (amax) over a particular
               time period (t). This bound could be obtained as vmax /
               t, where vmax is an upper bound on the average speed of
               the vehicle during that interval.

               The total acceleration of the vehicle, on Earth at
               least, is m = g + a, where a is the actual acceleration
               of the vehicle with |a| < amax. Note that g, a, and m
               are vector quantities. The vectors m and g are most
               different when a is perpendicular to g. The angle
               between the vectors m and g is then

               \theta_error = atan2(amax, |g|).

               In other words, the gravity vector 'g' lies within a
               cone whose point is at the origin, whose base is
               centered at 'm', and the angle between the central axis
               (origin to 'm') and any point on its surface is
               \theta_error.

               This assumes that the accelerations are measured
               without noise, and that the accelerometers themselves
               do not rotate during the interval t. Noise in the
               accelerometers can be accounted for by increasing amax,
               but rotations can cause the measured acceleration m to
               be significantly off.

               Consider the hypothetical case where the sensors rotate
               ninety degrees half-way through the time period t. With
               no additional linear acceleration, the measured vector
               will have magnitude |g|*sqrt(2)/2. Note that this is
               less than |g|, and in general, the integral of the
               gravity vector is maximized only when the platform is

               The total acceleration of the vehicle is then within
               amax of g (i.e., its magnitude is between g-amax and
               g+amax.) Suppose our accelerometers observe an
               acceleration m. We wish to determine how close to g 'm'
               is: naturally, if amax is small, then m is a good
               estimate of the gravity vector.

               Another way of viewing the problem is that we must be
               able to account for up to |g| + amax acceleration. When
               we observe |m|, the uncertainty in acceleration is the
               balance: |g| + |amax| - |m|. This acceleration could be
               in any direction.

               We now wish to find the cone in which the gravity
               vector 'g' lies given 'm'. The worst case is when the
               unaccounted-for acceleration is perpendicular to
               'm'. The resulting angle is:

               \theta_error = atan2(|g| + |amax| - |m|, |m|).

               Note that in the case where 'm' is 'g', this equation
               is identical to the equation above. It is more
               conservative than the previous case: if |m| = |g| +
               amax, we could deduce that |m| points exactly in the
               direction of g, since the only way the vehicle could
               have a net acceleration this large is if the two
               accelerations were aligned exactly. (The same holds is
               if |m| is smaller than |g|). The second equation does
               not exploit this knowledge, but is more robust in the
               presence of rotational errors.

               Note that if |m| < |g| + |amax| - |m|, the unknown
               acceleration is greater than the observed
               acceleration. This means that the direction of 'g' does
               not lie within any cone around 'm': 'g' could in fact
               be in any direction.
            */

            double dt = (d.utime_pimu - lastData.utime_pimu) / 1.0E6;

            // project our previous accelerations into our current
            // orientation and accumulate
            accel_acc = LinAlg.add(LinAlg.scale(accel, dt), LinAlg.transform(LinAlg.inverse(dT), accel_acc));
            accel_acc_t += dt;

            if (true) {

                // a bound on our average acceleration over the integration period (m/s^2)
                double maxa = maxv / accel_acc_t;

                double z[] = LinAlg.scale(accel_acc, 1.0 / accel_acc_t);
                // the accel vector points towards gravity plus or minus what angle?
                double z_mag = LinAlg.magnitude(z);
                z = LinAlg.normalize(z);

                if (z_mag*G > G + maxa - z_mag*G) {

                    double up[] = new double[] { 0, 0, 1};
                    double zglobal[] = LinAlg.transform(T, z);

                    // what quaternion would cause our observed up vector to actually point up?
                    double q[] = LinAlg.quatCompute(zglobal, up);
                    double aa[] = LinAlg.quatToAngleAxis(q);

                    double thetaUncertainty = Math.abs(Math.atan2(G + maxa - z_mag*G, z_mag*G));
                    last_up_uncertainty = thetaUncertainty;

                    if (Math.abs(aa[0]) > thetaUncertainty) {

                        if (aa[0] > 0)
                            aa[0] -= thetaUncertainty;
                        else
                            aa[0] += thetaUncertainty;

                        q = LinAlg.angleAxisToQuat(aa);
                        T = LinAlg.matrixAB(LinAlg.quatToMatrix(q), T);

                        // don't need to adjust accel_acc because it's
                        // maintained in IMU-relative coordinates.
                   }
                }

                if (accel_acc_t > maxintperiod) {
                    double shrink = 0.1;
                    accel_acc_t = shrink * accel_acc_t;
                    accel_acc = LinAlg.scale(accel_acc, shrink);
                }
            }
        }

        lastData = d;
    }
}
