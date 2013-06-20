package probcog.sensor;

import java.awt.*;
import java.awt.image.*;
import javax.swing.*;
import java.util.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.util.*;
import april.vis.*;
import april.vis.VisCameraManager.CameraPosition;

/** Provides KinectSensor-like access to the 3D world. Can return
 *  colors and points for objects in the world. Points that do not
 *  make up objects are automatically returned as transparent points
 *  at the origin.
 */
public class SimKinectSensor implements Sensor
{
    // Sim Kinect parameters
    public static final int WIDTH = 320;
    public static final int HEIGHT = (int)(WIDTH*.75);
    public static final double HFOV = 57.0;
    public static final double VFOV = 43.0;

    CameraPosition camera = new CameraPosition();

    SimWorld sw;

    JFrame jf;
    JImage jim;
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    /** Takes as input the SimWorld from which point data is generated */
    public SimKinectSensor(SimWorld sw_)
    {
        if (false) {
            jf = new JFrame("DEBUG WINDOW");
            jf.setSize(WIDTH,HEIGHT);
            jim = new JImage(WIDTH,HEIGHT);
            jim.setFlipY(true);
            jf.add(jim);
            //jf.add(vc);
            jf.setVisible(true);
        }

        sw = sw_;

        vw = new VisWorld();
        vl = new VisLayer(vw);

        // Set up the kinect view. Anchored to a fixed point. Sets this up
        // as our VisLayer's default view, too, so when we render for the
        // canvas, our view of the world will be accurate
        camera.eye = new double[] {0.6, 0, 1.0};    // Camera position
        camera.lookat = new double[3];              // Looks at origin
        camera.up = new double[] {-1.0, 0, 1.0};    // Up vector
        //camera.eye = new double[] {1, 1, 1};
        //camera.lookat = new double[] {0, 0, 0};
        //camera.up = new double[] {-1, -1, 1};

        camera.perspective_fovy_degrees = VFOV;
        camera.layerViewport = new int[] {0,0,WIDTH,HEIGHT};

        DefaultCameraManager cm = new DefaultCameraManager();

        cm.UI_ANIMATE_MS = 0;
        cm.BOOKMARK_ANIMATE_MS = 0;
        cm.FIT_ANIMATE_MS = 0;
        cm.interfaceMode = 3.0;

        vl.cameraManager = cm;
        vl.cameraManager.goBookmark(camera);
        vl.backgroundColor = Color.white;

        vc = new VisCanvas(vl);
        vc.setSize(WIDTH, HEIGHT);

        (new RenderThread()).start();
    }

    class RenderThread extends Thread
    {
        int Hz = 30;
        public void run()
        {
            while (true) {
                VisWorld.Buffer vb = vw.getBuffer("objs");
                for (SimObject obj: sw.objects) {
                    vb.addBack(new VisChain(obj.getPose(),
                                            obj.getVisObject()));
                }
                vb.swap();
                // I don't feel like this guarantees that we'll have our image data in
                // time for color sampling...
                vc.draw();
                TimeUtil.sleep(1000/Hz);
            }
        }
    }

    /** Return the real-world position of the camera */
    public double[][] getCameraXform()
    {
        double[][] camMatrix = new double[4][4];
        // Kinect axes
        double[] z = LinAlg.normalize(LinAlg.subtract(camera.lookat, camera.eye));
        double[] y = LinAlg.normalize(LinAlg.scale(camera.up, -1));
        double[] x = LinAlg.normalize(LinAlg.crossProduct(y, z));
        y = LinAlg.normalize(LinAlg.crossProduct(z, x));

        camMatrix[0][0] = x[0];
        camMatrix[0][1] = y[0];
        camMatrix[0][2] = z[0];
        camMatrix[0][3] = camera.eye[0];
        camMatrix[1][0] = x[1];
        camMatrix[1][1] = y[1];
        camMatrix[1][2] = z[1];
        camMatrix[1][3] = camera.eye[1];
        camMatrix[2][0] = x[2];
        camMatrix[2][1] = y[2];
        camMatrix[2][2] = z[2];
        camMatrix[2][3] = camera.eye[2];
        camMatrix[3][3] = 1.0;

        return camMatrix;
    }

    /** Get the RGBXYZ point corresponding to virtual kinect pixel (ix,iy) */
    public double[] getXYZRGB(int ix, int iy)
    {
        // XXX Infinite loop possibility
        BufferedImage im;
        do {
            im = vc.getLatestFrame();
            if (im == null)
                TimeUtil.sleep(10);
        } while (im == null);

        if (false) {
            jim.setImage(im);
        }

        // XXX Currently only supports 3BYTE BGR
        byte[] buf = ((DataBufferByte)(im.getRaster().getDataBuffer())).getData();
        //System.out.println(im.getWidth() + " " + im.getHeight() + " " + buf.length);

        // Find the ray leaving the camera
        GRay3D ray = camera.computeRay(ix, iy);

        // Search through the objects in the world and try to find the first
        // point at which the ray collides with one of these objects.
        // Apply color as needed, afterwards, defaulting otherwise to white.
        // If there is no point found, return all zeros
        double minDist = Double.MAX_VALUE;
        SimObject minObj = null;
        synchronized (sw) {
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
        }

        if(minObj == null) {
            double[] xyFloor = ray.intersectPlaneXY();
            minDist = LinAlg.distance(ray.getSource(), xyFloor);
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
        int idx = iy*WIDTH + ix;
        int b = buf[3*idx + 0];
        int g = buf[3*idx + 1];
        int r = buf[3*idx + 2];
        xyzc[3] = 0xff000000 |
                  (b & 0xff) |
                  (g & 0xff) << 8 |
                  (r & 0xff) << 16;

        return xyzc;
    }

    public int getWidth()
    {
        return WIDTH;
    }

    public int getHeight()
    {
        return HEIGHT;
    }

    public boolean stashFrame()
    {
        return true;
    }
}
