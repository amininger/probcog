package probcog.sensor;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import java.io.*;
import javax.swing.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;
import april.vis.*;

public class KinectCalibrator
{
    // Set point mode
    Mode mode = Mode.ORIG;
    enum Mode {
        ORIG, X, Y, TEST;
    }

    Config config;

    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;
    ParameterGUI pg;

    // Kinect
    Config config_;
    KinectSensor kinect;
    BufferedImage im = null;
    april.jmat.geom.Polygon poly;

    // Calibration points
    double[] origin = null;
    double[] px = null;
    double[] py = null;
    double[] pt = null;

    double x_offset = 0;

    public KinectCalibrator(Config config_) throws IOException
    {
        config = config_;

        JFrame jf = new JFrame("Kinect Calibrator");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        kinect = new KinectSensor(config_);
        // Load in arm parameter
        Config armConfig = new ConfigFile(config_.getPath("robot.arm"));
        String name = armConfig.getString("arm.arm_version");
        x_offset = armConfig.getDouble("arm."+name+".calib_offset", 0);

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        vl.addEventHandler(new ClickEventHandler());
        jf.add(vc, BorderLayout.CENTER);

        pg = new ParameterGUI();
        pg.addButtons("origin", "Set Origin",
                      "x-axis", "Set X-direction",
                      "y-axis", "Set Y-axis",
                      "calib",  "Calibrate",
                      "test",   "Set Test Point");
        pg.addButtons("frame",  "New Frame");
        pg.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name) {
                if (name.equals("origin")) {
                    mode = Mode.ORIG;
                } else if (name.equals("x-axis")) {
                    mode = Mode.X;
                } else if (name.equals("y-axis")) {
                    mode = Mode.Y;
                } else if (name.equals("calib")) {
                    if (origin != null && px != null && py != null)
                        saveCalibration();
                    else
                        System.err.println("ERR: Can only save if you define "+
                                           "calibration points");
                } else if (name.equals("test")) {
                    mode = Mode.TEST;
                } else if (name.equals("frame")) {
                    if (kinect.stashFrame()) {
                        updateImage();
                        poly = kinect.getPoly();
                        poly = flipPoly();
                        updateWindow();
                    }
                }
            }
        });
        jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }

    /** Construct a calibration and save it to a default file */
    private void saveCalibration()
    {
        System.out.println("Creating calibration...");

        if (im == null) {
            System.out.println("ERR: No image taken");
            return;
        }

        // Get kinect points. Remember to flip Y values!
        double[] ko = kinect.getLocalXYZ((int)origin[0],
                                         im.getHeight() - (int)origin[1]);
        double[] kx = kinect.getLocalXYZ((int)px[0],
                                         im.getHeight() - (int)px[1]);
        double[] ky = kinect.getLocalXYZ((int)py[0],
                                         im.getHeight() - (int)py[1]);

        //System.out.printf("\tO: [%f, %f, %f]\n", ko[0], ko[1], ko[2]);
        //System.out.printf("\tX: [%f, %f, %f]\n", kx[0], kx[1], kx[2]);
        //System.out.printf("\tY: [%f, %f, %f]\n", ky[0], ky[1], ky[2]);

        // Use the specified coordinates to define a world frame in terms
        // of kinect coordinates
        double[] wx = LinAlg.normalize(LinAlg.subtract(kx, ko));
        double[] wy = LinAlg.normalize(LinAlg.subtract(ky, ko));
        double[] wz = LinAlg.crossProduct(wx, wy);
        wy = LinAlg.crossProduct(wz, wx);

        // Translation of kinect origin to world origin
        double[][] k2wTranslate = new double[][] {
            {     1,     0,      0,  0},
            {     0,     1,      0,  0},
            {     0,     0,      1,  0},
            {-ko[0], -ko[1], -ko[2], 1}
        };

        // Transform from between world and kinect basis
        double[][] w2kTransform = new double[][] {
            {wx[0], wx[1], wx[2], 0},
            {wy[0], wy[1], wy[2], 0},
            {wz[0], wz[1], wz[2], 0},
            {    0,     0,     0, 1}
        };

        // Final adjustment in world coordinates to the actual arm origin
        double[][] wTranslate = new double[][] {
            {       1, 0, 0, 0},
            {       0, 1, 0, 0},
            {       0, 0, 1, 0},
            {x_offset, 0, 0, 1}
        };

        // Multiply out the matrices to provide a tranformation from kinect
        // coordinates to world coodinates w.r.t. the robot arm
        double[][] k2wTransform = LinAlg.matrixAB(LinAlg.matrixAB(k2wTranslate,
                                                                  LinAlg.inverse(w2kTransform)),
                                                  wTranslate);

        // Write to a configuration file
        String filename = config.getPath("kinect.calib_robot");
        if (filename == null)
            filename = "k2w.config";
        try {
            BufferedWriter fout = new BufferedWriter(new FileWriter(filename));
            System.out.println("Writing to calibration file "+filename);

            // Save transformation matrix
            // XXX Should really be able to write to a matrix, not flat array
            fout.write("calibration {\n");
            fout.write("\tk2w = [ ");
            for (int i = 0; i < 4; i++) {
                for (int j = 0; j < 4; j++) {
                    if (i == 3 && j == 3)
                    {
                        fout.write(k2wTransform[i][j] + " ];\n");
                    } else {
                        fout.write(k2wTransform[i][j] + ", ");
                    }
                }
            }
            fout.write("\n");

            // Write out polygon. Don't forget to flip y values!
            // XXX Also should be a matrix, not a flat array
            fout.write("\tdim = [ "+im.getWidth()+", "+im.getHeight()+" ];\n");
            fout.write("\tpoly = [ ");
            ArrayList<double[]> points = poly.getPoints();
            for (int i = 0; i < points.size(); i++) {
                if (i+1 == points.size()) {
                    fout.write(points.get(i)[0]+", "+
                               (im.getHeight() - points.get(i)[1])+" ];\n");
                } else {
                    fout.write(points.get(i)[0]+", "+
                               (im.getHeight() - points.get(i)[1])+", ");
                }
            }
            fout.write("}\n");
            fout.close();
        } catch (IOException ioex) {
            System.err.println("ERR: Could not write out calibration file");
            ioex.printStackTrace();
            return;
        }

        // Reload the kinect with our new parameters
        try {
            System.out.println("Trying to construct new KinectSensor");
            //calib_robot = new ConfigFile(filename); // XXX
            kinect = new KinectSensor(config);

            int timeout = 10;
            while (timeout > 0 && !kinect.stashFrame()) {
                TimeUtil.sleep(1000/30);
                timeout--;
            }
            if (timeout > 0) {
                System.out.println("updating frame...");
                updateImage();
                poly = kinect.getPoly();
                poly = flipPoly();
                updateWindow();
            } else {
                System.err.println("ERR: Failed to load new frame from kinect");
                im = null;
                vw.getBuffer("image").swap();
            }
        } catch (IOException ioex) {
            System.err.println("ERR: Could not open k2w.config");
            ioex.printStackTrace();
        }

        System.out.println("DONE!");
    }

    /** Render the currently stored Kinect image */
    private void updateImage()
    {
        VisWorld.Buffer vb = vw.getBuffer("image");
        vb.setDrawOrder(-100);
        im = kinect.getImage();
        if (im == null)
            return;
        vb.addBack(new VzImage(im, VzImage.FLIP));
        vb.swap();

        vl.cameraManager.fit2D(new double[2],
                               new double[] {im.getWidth(), im.getHeight()},
                               true);
    }

    /** Render the points designated by the calibration user */
    private void updatePoints()
    {
        VisWorld.Buffer vb = vw.getBuffer("points");
        vb.setDrawOrder(100);
        if (origin != null) {
            vb.addBack(new VisChain(LinAlg.translate(origin),
                                    new VzCircle(4,
                                                 new VzLines.Style(Color.red, 2))));
        }

        if (px != null) {
            vb.addBack(new VisChain(LinAlg.translate(px),
                                    new VzCircle(4,
                                                 new VzLines.Style(Color.green, 2))));
        }

        if (py != null) {
            vb.addBack(new VisChain(LinAlg.translate(py),
                                    new VzCircle(4,
                                                 new VzLines.Style(Color.yellow, 2))));
        }

        if (pt != null) {
            vb.addBack(new VisChain(LinAlg.translate(pt),
                                    new VzCircle(4,
                                                 new VzLines.Style(Color.blue, 2))));
        }

        vb.swap();
    }

    /** Render the viewing window */
    private void updateWindow()
    {
        if (im == null)
            return;
        VisWorld.Buffer vb = vw.getBuffer("window");
        vb.setDrawOrder(50);
        if (poly == null) {
            int w = im.getWidth();
            int h = im.getHeight();
            ArrayList<double[]> points = new ArrayList<double[]>();
            points.add(new double[2]);
            points.add(new double[]{0,h});
            points.add(new double[]{w,h});
            points.add(new double[]{w,0});
            poly = new april.jmat.geom.Polygon(points);
        }

        VisVertexData vvd = new VisVertexData(poly.getPoints());
        vb.addBack(new VzPoints(vvd, new VzPoints.Style(Color.cyan, 6)));
        vb.addBack(new VzLines(vvd,
                               VzLines.LINE_LOOP,
                               new VzLines.Style(Color.white, 2)));

        vb.swap();
    }

    // === EVENT HANDLING =============
    class ClickEventHandler extends VisEventAdapter
    {
        int selection = -1;
        /** User clicks allow the user to place various points
         *  on the world for calibration
         **/
        public boolean mouseClicked(VisCanvas vc,
                                    VisLayer vl,
                                    VisCanvas.RenderInfo rinfo,
                                    GRay3D ray,
                                    MouseEvent e)
        {
            if (e.getButton() != MouseEvent.BUTTON1)
                return false;

            double[] xy = ray.intersectPlaneXY();
            xy[0] = Math.floor(xy[0]) + 0.5;
            xy[1] = Math.floor(xy[1]) + 0.5;

            if (im == null)
                return false;

            if (xy[0] < 0 || xy[0] >= im.getWidth() ||
                xy[1] < 0 || xy[1] >= im.getHeight())
            {
                return false;
            }

            switch (mode) {
                case ORIG:
                    origin = xy;
                    break;
                case X:
                    px = xy;
                    break;
                case Y:
                    py = xy;
                    break;
                case TEST:
                    pt = xy;
                    // Print real-world coordinates based on most recent cal
                    double[] wt = kinect.getXYZ((int)pt[0],
                                                im.getHeight() - (int)pt[1],
                                                false);
                    if (wt != null)
                        System.out.printf("[%f, %f, %f]\n", wt[0], wt[1], wt[2]);

                    break;
                default:
                    System.err.println("ERR: No such mode");
            }

            updatePoints();
            return true;
        }

        /** Select boundary points of the view polygen when applicable */
        public boolean mousePressed(VisCanvas vc,
                                    VisLayer vl,
                                    VisCanvas.RenderInfo rinfo,
                                    GRay3D ray,
                                    MouseEvent e)
        {
            if (e.getButton() != MouseEvent.BUTTON1)
                return false;

            if (poly == null)
                return false;

            double[] xy = LinAlg.resize(ray.intersectPlaneXY(), 2);
            double thresh = 5.0;


            // Check to see if the ray intersects any of our poly points
            ArrayList<double[]> points = poly.getPoints();
            for (int i = 0; i < points.size(); i++) {
                if (LinAlg.distance(points.get(i), xy) < thresh) {
                    selection = i;
                    break;
                }
            }

            return false;
        }

        public boolean mouseReleased(VisCanvas vc,
                                     VisLayer vl,
                                     VisCanvas.RenderInfo rinfo,
                                     GRay3D ray,
                                     MouseEvent e)
        {
            if (e.getButton() != MouseEvent.BUTTON1)
                return false;

            selection = -1;

            return false;
        }

        /** Use mouse drags on boundary points of the view polygon move
         *  them around, changing the view window.
         *  XXX Eventually, this will be a sexy transparent overlay much like
         *  cropping looks in GIMP, etc
         **/
        public boolean mouseDragged(VisCanvas vc,
                                    VisLayer vl,
                                    VisCanvas.RenderInfo rinfo,
                                    GRay3D ray,
                                    MouseEvent e)
        {

            if (selection < 0 || selection > 3)
                return false;

            double[] xy = LinAlg.resize(ray.intersectPlaneXY(), 2);

            if (im == null || poly == null)
                return false;

            if (xy[0] < 0 || xy[0] >= im.getWidth() ||
                xy[1] < 1 || xy[1] >= im.getHeight())
            {
                return true;
            }

            poly.getPoints().set(selection, xy);

            updateWindow();

            return true;
        }
    }

    // Need to do a deep copy!
    private april.jmat.geom.Polygon flipPoly()
    {
        assert (im != null && poly != null);

        ArrayList<double[]> points = poly.getPoints();
        ArrayList<double[]> copyPoints = new ArrayList<double[]>();
        for (double[] p: points) {
            double[] cp = LinAlg.copy(p);
            cp[1] = im.getHeight() - cp[1];
            copyPoints.add(cp);
        }

        return new april.jmat.geom.Polygon(copyPoints);
    }

    // ================================


    /** A kinect calibration GUI which allows the user to click on points in
     *  a frame of kinect data to specify how those points correspond to points
     *  in the arm's coordinate frame. A config file input provides necessary
     *  access to information such as what arm is in use, allowing the
     *  calibrator to adjust its origin position accordingly.
     **/
    public static void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Input configuration file");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing arguments - "+opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help") ||
            opts.getString("config") == null)
        {
            opts.doHelp();
            System.exit(0);
        }

        Config config = null;
        try {
            config = new ConfigFile(opts.getString("config"));
        } catch (IOException ioex) {
            System.err.println("ERR: Error opening config file");
            ioex.printStackTrace();
            System.exit(1);
        }

        try {
            KinectCalibrator kc = new KinectCalibrator(config);
        } catch (IOException ioex) {
            System.err.println("ERR: Could not launch calibrator");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}
