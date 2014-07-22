package probcog.perception;

import java.io.IOException;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.lcmtypes.laser_t;
import april.util.*;
import april.vis.VisCameraManager.CameraPosition;

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

        MapThread mt = new MapThread();
        mt.start();
    }

    public ObstacleMap(Sensor sensor)
    {
        this.sensor = sensor;
        MapThread mt = new MapThread();
        mt.start();
    }

    public void createMap2D(double minHeight, double maxHeight)
    {

        if (!sensor.stashFrame())
            return;

        int width = sensor.getWidth();
        int height = sensor.getHeight();


        minHeight = Math.abs(minHeight); // Make sure min height is a positive number

        double[] radii = new double[width];
        for(int i=0; i<radii.length; i++) {
            radii[i] = Double.MAX_VALUE;
        }

        SimKinectSensor kinect = (SimKinectSensor) sensor;
        CameraPosition camera = kinect.camera;
        double[] eye = camera.eye;
        double[] up = camera.up;
        double[] lookat = camera.lookat;
        System.out.printf("Eye : (%.2f, %.2f, %.2f)\n", eye[0], eye[1], eye[2]);
        System.out.printf("Look: (%.2f, %.2f, %.2f)\n", lookat[0], lookat[1], lookat[2]);
        System.out.printf("Up  : (%.2f, %.2f, %.2f)\n", up[0], up[1], up[2]);

        for(int x=0; x<width; x++) {
            for(int y=0; y<height; y++) {

                double[] p = sensor.getXYZRGB(x,y);

                if(!isEmpty(p) && (p[2] < maxHeight && p[2] > minHeight))
                {
                    double[] xy = new double[]{p[0], p[1]};
                    double r = LinAlg.magnitude(xy);

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

        publishLaser(radiif, (float) Math.toRadians(57.0/2), (float) -(Math.toRadians(57.0)/(sensor.getWidth()-1)));
    }

    private boolean isEmpty(double[] p)
    {
        double epsilon = .0001;
        for(int i=0; i<3; i++) {
            if(Math.abs(p[i]) > epsilon)
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
            Config config = Util.getConfig();
            minHeight = config.requireDouble("obstacle.min_height");
            maxHeight = config.requireDouble("obstacle.max_height");
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
