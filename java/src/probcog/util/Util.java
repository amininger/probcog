package probcog.util;

import java.util.*;

import april.jmat.*;
import april.vis.VisCameraManager.CameraPosition;

import probcog.sensor.*;

public class Util
{
    /** Give a string of key=token pairs, extract the token
     *  value corresponding to the given key
     **/
    public static String getTokenValue(String params, String tokenKey)
    {
        String[] tokens = params.split(",");
        for (String token: tokens) {
            String[] keyValuePair = token.split("=");
            if (keyValuePair.length < 2)
                continue;
            if (keyValuePair[0].equals(tokenKey))
                return keyValuePair[1];
        }

        return null;    // No token found
    }

    public static ArrayList<String> getPossibleValues(String[] pairs, String key)
    {
        ArrayList<String> values = new ArrayList<String>();
        for (String pair: pairs) {
            String[] keyValuePair = pair.split("=");
            if (keyValuePair[0].equals(key))
                values.add(keyValuePair[1]);
        }

        return values;
    }

    public static String nextValue(ArrayList<String> values, String value)
    {
        for (int i = 0; i < values.size(); i++) {
            if (values.get(i).equals(value))
                return values.get((i+1)%values.size());
        }
        return null;
    }

    static int id = 0;
    public static int nextID()
    {
        return id++;
    }

    public static boolean equals(double a, double b, double thresh)
    {
        return Math.abs(a-b) < thresh;
    }

    public static double[] toArray(Collection<Double> collection, double[] da)
    {
        if (da == null || da.length < collection.size()) {
            da = new double[collection.size()];
        }
        int i = 0;
        for (Double d: collection) {
            da[i++] = d;
        }

        return da;
    }

    public static int[] toArray(Collection<Integer> collection, int[] ia)
    {
        if (ia == null || ia.length < collection.size()) {
            ia = new int[collection.size()];
        }
        int j = 0;
        for (Integer i: collection) {
            ia[j++] = i;
        }

        return ia;
    }

    public static ArrayList<double[]> extractPoints(Sensor sensor)
    {
        ArrayList<double[]> points = new ArrayList<double[]>();
        int height = sensor.getHeight();
        int width = sensor.getWidth();

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                double[] xyzrgb = sensor.getXYZRGB(x,y);
                if (xyzrgb == null)
                    continue;
                //System.out.printf("%f %f %f\n", xyzrgb[0], xyzrgb[1], xyzrgb[2]);

                points.add(xyzrgb);
            }
        }
        return points;
    }

    public static CameraPosition getSensorPos(Sensor sensor)
    {
        double[][] camMatrix = sensor.getCameraXform();

        CameraPosition camera = new CameraPosition();
        camera.eye = new double[3];
        camera.eye[0] = camMatrix[0][3];
        camera.eye[1] = camMatrix[1][3];
        camera.eye[2] = camMatrix[2][3];
        camera.layerViewport = new int[4];

        double[] sx = new double[] {camMatrix[0][0],
                                    camMatrix[1][0],
                                    camMatrix[2][0]};
        double[] sy = new double[] {camMatrix[0][1],
                                    camMatrix[1][1],
                                    camMatrix[2][1]};
        double[] sz = new double[] {camMatrix[0][2],
                                    camMatrix[1][2],
                                    camMatrix[2][2]};

        // Lets us account for weird coordinate frames
        if (sensor instanceof KinectSensor ||
            sensor instanceof SimKinectSensor)
        {
            camera.lookat = LinAlg.add(camera.eye, sz);
            camera.up = LinAlg.scale(sy, -1);
            camera.layerViewport[2] = sensor.getWidth();
            camera.layerViewport[3] = sensor.getHeight();
            camera.perspective_fovy_degrees = SimKinectSensor.VFOV;
        } else {
            camera.lookat = new double[3];
            camera.up = new double[3];
        }

        return camera;
    }

    static public void printCamera(CameraPosition camera)
    {
        System.out.printf("eye:    [%2.3f, %2.3f, %2.3f]\n"+
                          "lookat: [%2.3f, %2.3f, %2.3f]\n"+
                          "up:     [%2.3f, %2.3f, %2.3f]\n",
                          camera.eye[0], camera.eye[1], camera.eye[2],
                          camera.lookat[0], camera.lookat[1], camera.lookat[2],
                          camera.up[0], camera.up[1], camera.up[2]);
    }

    static public void main(String[] args)
    {
        System.out.printf("%d %d %d\n", nextID(), nextID(), nextID());
    }
}
