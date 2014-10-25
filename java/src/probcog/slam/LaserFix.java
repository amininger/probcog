package probcog.slam;

import java.util.*;

import april.config.*;
import april.jmat.*;
import april.util.*;

import probcog.sensor.*;
import probcog.util.*;
// import probcog.lcmtypes.*;

import magic2.lcmtypes.*;

public class LaserFix
{
    Odometry odometry = new Odometry();

    ArrayList<Data> sweep = new ArrayList<Data>();
    double last_position_radians = 0;
    int last_direction = 1;
    static final int MAX_SWEEP_SLICES = 200;

    // GridMap constants
    static final double MPP = 0.05;      // [meters/px]
    static final double SIZE_X = 20.0;  // [meters]
    static final double SIZE_Y = 20.0;  // [meters]
    double minHeight = .2; // XXX Should be smaller, but then we see the floor, probably wrong dynamixel association
                           //Util.getConfig().requireDouble("obstacle.min_height");
    double maxHeight = 1.0;//Util.getConfig().requireDouble("obstacle.max_height");


    public static class Data
    {
        public pose_t pose;
        public dynamixel_status_t status;
        public laser_t laser;
    }

    public LaserFix()
    {
    }

    public laser_t getFixedLaserT(pose_t pose)
    {
        GridMap gm = processDatas(pose, sweep); // XXX Need this from the old lidar data

        laser_t laser = new laser_t();
        laser.rad0 = (float)Math.toRadians(-135);
        laser.radstep = (float)Math.toRadians(0.5);
        laser.nranges = (int)(2*Math.abs(laser.rad0/laser.radstep));
        laser.ranges = new float[laser.nranges];
        laser.utime = TimeUtil.utime();

        double[] xyt = LinAlg.quatPosToXYT(pose.orientation,
                                           pose.pos);

        // Expensive grid map processing
        float step = (float)gm.metersPerPixel/2;
        for (int i = 0; i < laser.nranges; i++) {
            laser.ranges[i] = -0.00001f;
            float theta = (float)(xyt[2]+laser.rad0+laser.radstep*i);
            double x = 0, y = 0;
            for (float r = 0.0f; r < 5.0f; r+=step) {
                x = xyt[0] + r*Math.cos(theta);
                y = xyt[1] + r*Math.sin(theta);
                if (gm.getValue(x, y) != 0) {
                    laser.ranges[i] = r;
                    break;
                }
            }
        }

        return laser;
    }


    private GridMap processDatas(pose_t pose, ArrayList<Data> datas)
    {
        GridMap gm = GridMap.makeMeters(pose.pos[0]-SIZE_X/2, pose.pos[1]-SIZE_Y/2, SIZE_X, SIZE_Y, MPP, 0);

        // For each sweep, add those points to the map when relevant. Anything
        // between min and max height gets added
        for (Data data: datas) {
            // Project the points

            // body to local frame
            double[][] B2L = LinAlg.quatPosToMatrix(data.pose.orientation, data.pose.pos);

            // sensor to body
            double q[] = LinAlg.rollPitchYawToQuat(new double[3]);
            double pos[] = new double[]{0.266, 0.0, 0.584};

            double[][] S2B = LinAlg.multiplyMany(LinAlg.quatPosToMatrix(q, pos),
                                                 //ConfigUtil.getRigidBodyTransform(Util.getConfig(), "HOKUYO_LIDAR"),
                                                 LinAlg.rotateY(-data.status.position_radians),
                                                 LinAlg.translate(0,0, 0.026));//Util.getConfig().requireDouble("HOKUYO_LIDAR.zoffset")));

            ArrayList<double[]> pointsRaw = new ArrayList<double[]>();
            for (int i = 0; i < data.laser.nranges; i++) {
                double r = data.laser.ranges[i];

                // Skip error codes
                if (r < 0)
                    continue;

                double t = data.laser.rad0 + data.laser.radstep*i;

                double[] xyz = new double[3];
                xyz[0] = r*Math.cos(t);
                xyz[1] = r*Math.sin(t);
                pointsRaw.add(xyz);

                // TODO: Glance points filter

                // Sensor to local frame
                double[][] S2L = LinAlg.multiplyMany(B2L, S2B);
                ArrayList<double[]> pointsLocal = LinAlg.transform(S2L, pointsRaw);


                // Put the points in the map
                for (double[] p: pointsLocal) {
                    // Skip non-obstacle points
                    if (p[2] < minHeight || p[2] > maxHeight)
                        continue;
                    gm.setValue(p[0], p[1], (byte)255);
                }

            }
        }
        return gm;
    }


    public void addData(pose_t pose, laser_t laser_hokuyo, dynamixel_status_t status)
    {
        Data d = new Data();
        d.laser = laser_hokuyo;
        d.pose = pose;
        d.status = status;

        int direction = DynamixelPoseLidar.sign(d.status.position_radians - last_position_radians);

        if (direction == last_direction) {
            if (sweep.size() < MAX_SWEEP_SLICES)
                sweep.add(d);
        } else {
            // is this a "full" sweep?
            sweep = new ArrayList<Data>();
            sweep.add(d);
        }

        last_direction = direction;
        last_position_radians = d.status.position_radians;
    }
}
