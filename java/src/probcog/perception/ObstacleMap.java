package probcog.perception;

import java.io.IOException;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.lcmtypes.laser_t;
import april.util.*;

import probcog.sensor.*;
import probcog.util.*;

public class ObstacleMap
{
    Sensor sensor;

    static LCM lcm = LCM.getSingleton();

    public ObstacleMap()
    {
        try {
            sensor = new KinectSensor();
        } catch (IOException ioex) {
            ioex.printStackTrace();
        }

        new MapThread().start();
    }

    public void createMap2D(double minHeight, double maxHeight)
    {

        if (!sensor.stashFrame())
            return;

        int width = sensor.getWidth();
        int height = sensor.getHeight();
        ArrayList<double[]> points = sensor.getAllXYZRGB();

        minHeight = Math.abs(minHeight); // Make sure min height is a positive number

        double[] radii = new double[width];

        for(int x=0; x<width; x++) {
            for(int y=0; y<height; y++) {

                double[] p = points.get(y*width + x);

                if(!isEmpty(p) &&
                   ((p[2] < maxHeight && p[2] > minHeight) ||
                    (p[2] < -minHeight))) {

                    double[] xy = new double[]{p[0], p[1]};
                    double r = LinAlg.distance(xy, new double[2]);

                    if(r < radii[x]) {
                        radii[x] = r;
                    }
                }

            }
        }

        float[] radiif = new float[radii.length];
        for(int i=0; i<radii.length; i++) {
            radiif[i] = (float) radii[i];
        }

        double[] p0 = points.get((int) (height/2.0)*width);
        double[] p1 = points.get((int) (height/2.0)*width+1);
        float thetaStart = (float) Math.atan2(p0[1], p0[0]);
        float thetaNext = (float) Math.atan2(p1[1], p1[0]);
        float thetaStep = (float) thetaNext - thetaStart;

        publishLaser(radiif, thetaStart, thetaStep);
    }

    private boolean isEmpty(double[] p)
    {
        double epsilon = .0001;
        for(double d : p) {
            if(Math.abs(d) > epsilon)
                return false;
        }
        return true;
    }

    private void publishLaser(float[] radii, float rStart, float rStep)
    {
        if (radii == null)
            return;

        laser_t laser = new laser_t();

        laser.nranges = radii.length;
        laser.ranges = radii;

        laser.nintensities = 0;

        laser.rad0 = rStart;
        laser.radstep = rStep;

        laser.utime = TimeUtil.utime();
        lcm.publish("LASER", laser);
    }


    class MapThread extends Thread
    {
        double minHeight, maxHeight;
        public MapThread()
        {
            Config config = Util.getDomainConfig();
            double minHeight = config.requireDouble("obstacle.min_height");
            double maxHeight = config.requireDouble("obstacle.max_height");
        }

        public void run()
        {
            while(true) {
                createMap2D(minHeight, maxHeight);
                TimeUtil.sleep(1000/30);
            }
        }
    }

    public static void main(String[] args)
    {
        ObstacleMap obMap = new ObstacleMap();
    }
}