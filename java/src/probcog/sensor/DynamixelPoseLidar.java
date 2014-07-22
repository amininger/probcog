package probcog.sensor;

import java.io.*;
import java.rmi.*;
import java.util.*;
import java.awt.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;
import april.lcmtypes.*;
import april.config.*;

import probcog.lcmtypes.*;
import probcog.servo.*;
//import magic.util.*;
//import magic.lcmtypes.*;


public class DynamixelPoseLidar implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    public PoseTracker ptracker = PoseTracker.getSingleton();
    public DynamixelTracker dynamixelTracker = new DynamixelTracker("DYNAMIXEL_STATUS_HOKUYO");

    int delay;

    LinkedList<laser_t> queue = new LinkedList<laser_t>();

    ArrayList<Listener> listeners = new ArrayList<Listener>();

    ArrayList<Data> sweep = new ArrayList<Data>();
    double last_position_radians = 0;
    int last_direction = 1;

    static final int MAX_SWEEP_SLICES = 200;

    /** @param delay By how many scans should lidar data be delayed?
     * If too short, we may not be have good Dynamixel or Pose estimates.
     * Recommended: at east 3. **/
    public DynamixelPoseLidar(int delay)
    {
        this.delay = delay;

        assert(delay >= 0 && delay <= 5);

        lcm.subscribe("HOKUYO_LIDAR", this); // XXX
    }


    public static class Data
    {
        public pose_t pose;
        public dynamixel_status_t status;
        public laser_t laser;

        public ArrayList<double[]> toGlobalPoints(Config config)
        {
            return toGlobalPoints(config, 3, false, false);
        }

        public ArrayList<double[]> toGlobalPoints(Config config, int dim)
        {
            return toGlobalPoints(config, dim, false, false);
        }

        public ArrayList<double[]> toGlobalPoints(Config config, int dim, boolean usenull)
        {
            return toGlobalPoints(config, dim, usenull, false);
        }

        /** @param dim Allocate the arrays a 'dim' dimensional. Useful
         * for making 4 dimensional points for use with
         * CachedColorizer.. usenull: add a 'null' to points when the
         * point would otherwise be filtered.
         **/
        public ArrayList<double[]> toGlobalPoints(Config config, int dim,
                                                  boolean usenull, boolean filterSingularity)
        {
            return DynamixelPoseLidar.toGlobalPoints(config, pose, status, laser, dim, usenull, filterSingularity);
        }
    }

    // Given the robot kinematics, the position of the robot, the hokuyo angle, and the laser measurements,
    // project the points into 3D.
    public static ArrayList<double[]> toGlobalPoints(Config config, pose_t pose, dynamixel_status_t status, laser_t laser,
                                                     int dim, boolean usenull, boolean filterSingularity)
    {
        assert(dim >= 3);

        double min_range = config.getDouble("HOKUYO_LIDAR.min_range", 0);
        double max_range = config.getDouble("HOKUYO_LIDAR.max_range", Double.MAX_VALUE);

        ArrayList<double[]> points = new ArrayList<double[]>();

        double S2B[][] = LinAlg.multiplyMany(ConfigUtil.getRigidBodyTransform(config, "HOKUYO_LIDAR"),
                                             LinAlg.rotateY(-status.position_radians));
        double B2G[][] = LinAlg.quatPosToMatrix(pose.orientation, pose.pos);

        double T[][] = LinAlg.multiplyMany(B2G, S2B);

        for (int i = 0; i < laser.nranges; i++) {
            double r = laser.ranges[i];
            if (r < min_range || r > max_range) {
                if (usenull)
                    points.add(null);
                continue;
            }

            double theta = laser.rad0 + laser.radstep*i;

            if (filterSingularity && Math.abs(Math.abs(theta)-Math.PI/2)<0.175)
            {
                if (usenull)
                    points.add(null);
                continue;
            }

            double xyz[] = new double[dim];
            xyz[0] = r*Math.cos(theta);
            xyz[1] = r*Math.sin(theta);
            double p[] = LinAlg.transform(T, xyz);

            if (dim >= 4 && laser.nranges == laser.nintensities)
                p[3] = laser.intensities[i];

            points.add(p);
        }

        return points;
    }

    public interface Listener
    {
        public void handleData(Data d);
        public void handleSweep(ArrayList<Data> ds);
    }

    public void addListener(Listener listener)
    {
        listeners.add(listener);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            laser_t las = new laser_t(ins);

            queue.add(las);

            if (queue.size() >= delay) {
                las = queue.removeFirst();

                Data d = new Data();
                d.laser = las;
                d.pose = ptracker.get(d.laser.utime);
                d.status = dynamixelTracker.get(d.laser.utime);

                if (d.pose != null && d.status != null) {
                    for (Listener listener : listeners)
                        listener.handleData(d);

                    int direction = sign(d.status.position_radians - last_position_radians);

//                    System.out.printf("%15f %15f\n", d.status.position_radians, d.status.speed);

                    if (direction == last_direction) {
                        if (sweep.size() < MAX_SWEEP_SLICES)
                            sweep.add(d);

                    } else {
                        // is this a "full" sweep?
                        double tmin = 10;
                        double tmax = -10;
                        double tzero = 10;

                        for (Data dd : sweep) {
                            tmin = Math.min(tmin, dd.status.position_radians);
                            tmax = Math.max(tmax, dd.status.position_radians);
                            tzero = Math.min(tzero, Math.abs(dd.status.position_radians));
                        }

                        if (tmin > -.7 || tmax < .2 || tzero > 0.1) {
                            if (sweep.size() > 5) {
                                // don't bother reporting cases where we just have some noise at the end of our travel.
                                System.out.printf("DynamixelPoseLidar rejecting bad sweep: %15f %15f %15f (%d scans)\n",
                                                  tmin, tmax, tzero, sweep.size());
                            }
                        } else {
                            for (Listener listener : listeners)
                                listener.handleSweep(sweep);
                        }

                        sweep = new ArrayList<Data>();
                        sweep.add(d);
                    }

                    last_direction = direction;
                    last_position_radians = d.status.position_radians;
                }
            }

        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }

    static final int sign(double v)
    {
        if (v >= 0)
            return 1;
        return -1;
    }
}
