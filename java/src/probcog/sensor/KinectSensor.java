package probcog.sensor;

import java.awt.*;
import java.awt.image.*;
import java.io.*;
import javax.swing.*;
import java.util.*;

import lcm.lcm.*;

import april.camera.*;
import april.camera.models.*;
import april.config.*;
import april.jmat.*;
import april.util.*;
import april.vis.*;

import probcog.lcmtypes.*;

/** Provides access to the frames taken by the kinect. Only
 *  keeps track of the most recently received frame from the
 *  kinect.
 **/
public class KinectSensor
{
    // FOR NOW, hardcoded values for depth  camera calibration, which will
    // take some extra work to get to.
    //
    // parameters for IR depth camera
    //public static double Firx = 5.7191759217862204e+02; // focal length
    //public static double Firy = 5.8760489958891026e+02; //

    //public static double Cirx = 3.37025048258259540e+02; // larger -> moves right
    //public static double Ciry = 2.4675008449138741e+02; // larger -> moves down

    //public static double Cirx = 3.2525048258259540e+02; // camera center in pixels
    //public static double Ciry = 2.4275008449138741e+02; //

    LCM lcm = LCM.getSingleton();

    Object kinectLock = new Object();
    kinect_status_t ks = null;

    // Calibration
    Config color;
    Config ir;
    View input;
    View output;
    Rasterizer rasterizer;

    double Cirx, Ciry, Firx, Firy;

    // Stash
    kinect_status_t stash_ks;
    BufferedImage r_rgbIm;
    BufferedImage r_depthIm;

    public KinectSensor(Config color_, Config ir_) throws IOException
    {
        color = color_;
        ir = ir_;

        // Set IR Paremeters
        Cirx = ir.requireDoubles("intrinsics.cc")[0];
        Ciry = ir.requireDoubles("intrinsics.cc")[1];
        Firx = ir.requireDoubles("intrinsics.fc")[0];
        Firy = ir.requireDoubles("intrinsics.fc")[1];

        // Create the input view
        System.err.println("NFO: Initializing kinect calibration");
        String classname = color.requireString("class");

        Object obj = ReflectUtil.createObject(classname, color);
        assert (obj != null);
        assert (obj instanceof Calibration);

        input = (Calibration) obj;

        // Create the output view. XXX Eventually specified by config or cmd?
        System.err.println("NFO: Initializing rectified view");
        output = new MaxRectifiedView(input);

        // Create the rasterizer. XXX Eventually specified by config or cmd?
        System.err.println("NFO: Initializing rasterizer");
        rasterizer = new NearestNeighborRasterizer(input, output);

        // Spin up LCM listener
        new ListenerThread().start();
    }

    //static int cnt = 0;
    class ListenerThread extends Thread implements LCMSubscriber
    {
        LCM lcm = LCM.getSingleton();

        public ListenerThread()
        {
            lcm.subscribe("KINECT_STATUS", this);
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/60);    // Just chewing up CPU time...
            }
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                messageReceivedEx(lcm, channel, ins);
            } catch (IOException ioex) {
                System.err.println("ERR: LCM channel ="+channel);
                ioex.printStackTrace();
            }
        }

        private void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
        {
            if (channel.equals("KINECT_STATUS")) {
                synchronized (kinectLock) {
                //    System.out.println(cnt++);
                    ks = new kinect_status_t(ins);
                }
            }
        }
    }

    /** "Stash" the current kinect frame data, which will then be
     *  used for all subsequent frame lookups. Calling stash again will
     *  replace the currently stashed frame with the most recent frame.
     *  This is also where data is undistorted so as to not waste cycles
     *  processing frames that will never be used.
     */
    public boolean stashFrame()
    {
        // Haven't received a new frame yet
        synchronized (kinectLock) {
            if (ks == null)
                return false;

            stash_ks = ks;
            ks = null;
        }

        // Undistort data
        BufferedImage rgbIm = new BufferedImage(stash_ks.WIDTH,
                                                stash_ks.HEIGHT,
                                                BufferedImage.TYPE_INT_RGB);
        BufferedImage depthIm = new BufferedImage(stash_ks.WIDTH,
                                                  stash_ks.HEIGHT,
                                                  BufferedImage.TYPE_INT_RGB);

        int[] brgb = ((DataBufferInt) (rgbIm.getRaster().getDataBuffer())).getData();
        int[] bdepth = ((DataBufferInt) (depthIm.getRaster().getDataBuffer())).getData();

        for (int y = 0; y < stash_ks.HEIGHT; y++) {
            for (int x = 0; x < stash_ks.WIDTH; x++) {
                int i = y*stash_ks.WIDTH + x;
                brgb[i] = 0xff000000 |
                          ((stash_ks.rgb[i*3 + 0] & 0xff) << 0) |
                          ((stash_ks.rgb[i*3 + 1] & 0xff) << 8) |
                          ((stash_ks.rgb[i*3 + 2] & 0xff) << 16);

                bdepth[i] = ((stash_ks.depth[i*2 + 0] & 0xff) << 0) |
                            ((stash_ks.depth[i*2 + 1] & 0xff) << 8);
            }
        }

        r_rgbIm = rasterizer.rectifyImage(rgbIm);
        r_depthIm = rasterizer.rectifyImage(depthIm);
        //r_rgbIm = rgbIm;
        //r_depthIm = depthIm;
        //
        //System.out.printf("%d x %d -- %d x %d\n", r_rgbIm.getWidth(),
        //                                          r_rgbIm.getHeight(),
        //                                          r_depthIm.getWidth(),
        //                                          r_depthIm.getHeight());

        return true;
    }

    /** Get the width of our rectified images */
    public int getWidth()
    {
        if (r_rgbIm == null)
            return 0;
        assert (r_rgbIm.getWidth() == r_depthIm.getWidth());

        return r_rgbIm.getWidth();
    }

    public int getHeight()
    {
        if (r_rgbIm == null)
            return 0;
        assert (r_rgbIm.getHeight() == r_depthIm.getHeight());

        return r_rgbIm.getHeight();
    }

    // XXX Add bounds checking
    /** Get the colored point at (ix, iy) in the stashed frame */
    public double[] getXYZRGB(int ix, int iy)
    {
        if (ix < 0 || ix >= r_rgbIm.getWidth() ||
            iy < 0 || iy >= r_rgbIm.getHeight())
            return null;

        double[] xyzc = getXYZ(ix, iy, new double[4]);
        xyzc[3] = getRGB(ix, iy);

        return xyzc;
    }

    /** Get the 3D point at (ix, iy) in the stashed frame */
    public double[] getXYZ(int ix, int iy)
    {
        return getXYZ(ix, iy, new double[3]);
    }

    public double[] getXYZ(int ix, int iy, double[] xyz)
    {
        if (ix < 0 || ix >= r_rgbIm.getWidth() ||
            iy < 0 || iy >= r_rgbIm.getHeight())
            return null;

        assert (xyz != null && xyz.length >= 3);
        assert (r_depthIm != null);
        int[] buf = ((DataBufferInt)(r_depthIm.getRaster().getDataBuffer())).getData();

        int d = buf[iy*r_depthIm.getWidth() + ix];
        double depth = d/1000.0;   // millimeters to meters

        xyz[0] = (ix - Cirx) * depth / Firx;
        xyz[1] = (iy - Ciry) * depth / Firy;
        xyz[2] = depth;

        return xyz;
    }

    /** Get the color of the point at (ix, iy) in the stashed frame */
    public int getRGB(int ix, int iy)
    {
        if (ix < 0 || ix >= r_rgbIm.getWidth() ||
            iy < 0 || iy >= r_rgbIm.getHeight())
            return 0;

        assert (r_rgbIm != null);
        int[] buf = ((DataBufferInt)(r_rgbIm.getRaster().getDataBuffer())).getData();

        return buf[iy*r_rgbIm.getWidth() + ix];
    }

    // === Debug GUI ==============
    public static void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h',"help",false,"Show this help screen");
        opts.addString('c',"color",null,"RGB calibration config");
        opts.addString('r',"ir",null,"IR calibration config");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args. "+opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help") ||
            opts.getString("color") == null ||
            opts.getString("ir") == null)
        {
            opts.doHelp();
            System.exit(0);
        }

        Config color = null;
        Config ir = null;
        try {
            color = new ConfigFile(opts.getString("color"));
            color = color.getChild("aprilCameraCalibration.camera0000");

            ir = new ConfigFile(opts.getString("ir"));
            ir = ir.getChild("aprilCameraCalibration.camera0000");
        } catch (IOException ioex) {
            System.err.println("ERR: Could not open calibration config");
            ioex.printStackTrace();
            System.exit(1);
        }

        JFrame jf = new JFrame("Kinect GUI");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800,600);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        jf.setVisible(true);

        KinectSensor kinect = null;
        try {
            // XXX HACK
            kinect = new KinectSensor(color, ir);
        } catch (IOException ioex) {
            System.err.println("ERR: Could not initialize KinectSensor");
            ioex.printStackTrace();
            System.exit(1);
        }

        int fps = 30;
        while (true) {
            TimeUtil.sleep(1000/fps);
            if (!kinect.stashFrame())
                continue;

            ArrayList<double[]> points = new ArrayList<double[]>();
            VisColorData vcd = new VisColorData();
            for (int y = 0; y < kinect.getHeight(); y++) {
                for (int x = 0; x < kinect.getWidth(); x++) {
                    double[] xyz = kinect.getXYZ(x,y);
                    int rgb = kinect.getRGB(x,y);

                    if (xyz == null)
                        continue;

                    points.add(xyz);
                    vcd.add(rgb);   // XXX probably flipped
                }
            }

            VisWorld.Buffer vb = vw.getBuffer("kinect");
            vb.addBack(new VzPoints(new VisVertexData(points),
                                    new VzPoints.Style(vcd, 2)));
            vb.swap();
        }
    }
}
