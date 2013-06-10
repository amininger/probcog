package probcog.sensor;

import java.util.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.vis.VisCameraManager.CameraPosition;

/** Provides KinectSensor-like access to the 3D world. Can return
 *  colors and points for objects in the world. Points that do not
 *  make up objects are automatically returned as transparent points
 *  at the origin.
 */
public class SimKinectSensor
{
    // Sim Kinect parameters
    public static final int WIDTH = 640;
    public static final int HEIGHT = 480;
    public static final double HFOV = 57.0;
    public static final double VFOV = 43.0;

    CameraPosition camera = new CameraPosition();

    SimWorld sw;

    /** Takes as input the SimWorld from which point data is generated */
    public SimKinectSensor(SimWorld sw_)
    {
        sw = sw_;

        camera.eye = new double[] {0.6, 0, 1.0};    // Camera position
        camera.lookat = new double[3];              // Looks at origin
        camera.up = new double[] {-1.0, 0, 0.6};    // Up vector

        camera.perspective_fovy_degrees = VFOV;      // XXX Maybe should be HFOV
        camera.layerViewport = new int[] {0,0,WIDTH,HEIGHT};
    }

    /** Get the RGBXYZ point corresponding to virtual kinect pixel (ix,iy) */
    public double[] getRGBXYZ(int ix, int iy)
    {
        // Find the ray leaving the camera
        GRay3D ray = camera.computeRay(ix, iy);

        // Search through the objects in the world and try to find the first
        // point at which the ray collides with one of these objects.
        // Apply color as needed, afterwards, defaulting otherwise to white.
        // If there is no point found, return all zeros
        double minDist = Double.MAX_VALUE;
        SimObject minObj = null;
        for (SimObject obj: sw.objects) {
            double dist = Collisions.collisionDistance(ray.getSource(),
                                                       ray.getDir(),
                                                       obj.getShape(),
                                                       obj.getPose());
            if (dist < minDist) {
                minDist = dist;
                minObj = obj;
            }
        }

        // Object is too far away for a kinect to sense anything. Return an
        // empty point
        if (minDist >= 8.0) {
            return new double[4];
        }

        // Compute the point in space we collide with the object at
        double[] xyzc = ray.getPoint(minDist);
        xyzc = LinAlg.resize(xyzc, 4);

        // If the object in question supports a color query, assign it
        // a color. Otherwise, default to white.
        xyzc[3] = 0xffffffff;
        // XXX COLOR CHECK

        return xyzc;
    }
}
