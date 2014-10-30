package probcog.slam;

import java.awt.*;
import java.awt.image.*;
import java.io.*;
import java.util.*;
import javax.swing.*;

import lcm.lcm.*;
import lcm.logging.*;

import april.config.*;
import april.graph.*;
import april.jmat.*;
import april.tag.*;
import april.util.*;
import april.vis.*;

// We have LCMTYPES in a lot of places. We should be able to do what we need
// for this iteration with stuff that is just in APRIL. Later, we probably want
// to switch over to MAGIC2 officially, but for now, avoid dependency. (Types
// unchanged for this application)
//import april.lcmtypes.*;    // laser_t, pose_t, tag_detection*
import magic2.lcmtypes.*;   // Must be magic2 for new tag_detections...

/** Take as input a robot log and create a maximum likelihood map. */
public class OfflineSLAM
{
    static final long STEP_TIME_US = 1*1000000;
    static final double ODOM_ERR_DIST = 0.15;   // STDDEV of err/m traveled
    static final double ODOM_ERR_DIST_FIXED = 0.100;
    static final double ODOM_ERR_ROT = 0.010;   // STDDEV of err/rad
    static final double ODOM_ERR_ROT_FIXED = 0.025;
    static final double TAG_ERR_TRANS = 0.25;
    static final double TAG_ERR_ROT = Math.PI/8;  // In radians

    static final boolean DRAW_GRIDMAP = false;
    double gridmap_size = 50;
    double metersPerPixel = .05;
    double rangeCovariance = 0.1;

    // Grab from config in future XXX
    double fx = 481.70;
    double fy = 482.65;
    double cx = 384.47;
    double cy = 213.876;
    double tagSize = 0.15;

    Config config;
    Config cal;

    // GUI Misc.
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;
    ParameterGUI pg;

    // Log state
    Log log;
    Odometry odometry = new Odometry();
    LaserFix laserFix = new LaserFix();
    TagFix tagFix;
    TagDetector detector;
    pose_t lastPose = null;
    dynamixel_status_t lastDynamixel = null;
    ArrayList<Integer> poseIdxs = new ArrayList<Integer>(); // In graph nodes
    ArrayList<pose_t> poses = new ArrayList<pose_t>();
    // XXX SYNCHRONIZATION IS AN ISSUE HERE...must examine LASER publishing code
    ArrayList<laser_t> lasers = new ArrayList<laser_t>();   // History of LASER?
    HashMap<Integer, Integer> tagIdxs = new HashMap<Integer, Integer>(); // In graph nodes
    HashMap<Integer, Set<Integer> > tag2pose = new HashMap<Integer, Set<Integer> >();
    ArrayList<tag_detection_list_t> tags = new ArrayList<tag_detection_list_t>();

    // Graph state
    boolean firstTime = false;
    GraphSolver solver;
    GraphSolver gssolver;
    Graph g;

    boolean drawing = false;
    private class DrawThread extends Thread
    {
        public void run()
        {
            redraw();
            drawing = false;
        }
    }

    private class ProcessThread extends Thread
    {
        boolean all = false;
        public ProcessThread(boolean all)
        {
            this.all = all;
        }

        public void run()
        {
            processLog(all);
        }
    }

    private class ButtonHandler implements ParameterListener
    {
        public void parameterChanged(ParameterGUI pg, String name)
        {
            if (name.equals("step")) {
                // Read up until the next node is added to the graph
                //processLog(false);
                new ProcessThread(false).start();
            } else if (name.equals("run")) {
                // Slurp up all of the log file
                //processLog(true);
                new ProcessThread(true).start();
            } else if (name.equals("opt")) {
                // Optimize the graph
                if (firstTime)
                    for (int i = 0; i < 10000; i++)
                        gssolver.iterate();
                solver.iterate();
                firstTime = false;
                redraw();
            }
        }
    }

    public OfflineSLAM(GetOpt opts, String logLocation)
    {
        initGUI();
        initGraph();

        try {
            log = new Log(logLocation, "r");
            config = new ConfigFile(opts.getString("config"));
            cal = new ConfigFile(opts.getString("cal"));
            tagFix = new TagFix(cal);
            initDetector();
        } catch (IOException ex) {
            System.err.println("ERR: Could not open LCM log "+logLocation);
            System.exit(-2);
        }
    }

    private void initGUI()
    {
        JFrame jf = new JFrame("OfflineSLAM");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(1000,800);

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        // XXX TODO: Hook up these buttons
        pg = new ParameterGUI();
        pg.addButtons("opt", "Optimize",
                      "step", "Step",
                      "run", "Run");
        pg.addListener(new ButtonHandler());
        jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }

    private void initGraph()
    {
        g = new Graph();

        GXYTNode n = new GXYTNode();
        n.state = new double[3];
        n.init = new double[3];
        n.truth = new double[3];    // We define our initial XYT to be 0.
        n.setAttribute("type", "robot");
        g.nodes.add(n);
        poseIdxs.add(g.nodes.size()-1);

        GXYTPosEdge e = new GXYTPosEdge();
        e.z = new double[3];
        e.truth = new double[3];
        e.P = new double[][] {{1e-6, 0, 0},
                              {0, 1e-6, 0},
                              {0, 0, 1e-6}};
        e.nodes = new int[] {0};
        e.setAttribute("type", "odom");
        g.edges.add(e);

        solver = new CholeskySolver(g);
        gssolver = new GaussSeidelSolver(g);
    }

    private void initDetector()
    {
        TagFamily tf = new Tag36h11();
        tf.setErrorRecoveryBits(config.requireInt("tag_detection.tag.errorBits"));
        detector = new TagDetector(tf);

        detector.debug = false;
        detector.sigma = config.requireDouble("tag_detection.tag.sigma");
        detector.segSigma = config.requireDouble("tag_detection.tag.segSigma");
        detector.segDecimate = config.requireBoolean("tag_detection.tag.decimate");
        detector.minMag = config.requireDouble("tag_detection.tag.minMag");
        detector.maxEdgeCost = Math.toRadians(config.requireDouble("tag_detection.tag.maxEdgeCost_deg"));
        detector.magThresh = config.requireDouble("tag_detection.tag.magThresh");
        detector.thetaThresh = config.requireDouble("tag_detection.tag.thetaThresh");
        detector.WEIGHT_SCALE = config.requireInt("tag_detection.tag.weightScale");
    }

    int counter = 0;
    /** Process the log. If all == true, read in the entire log immediately.
     *  Otherwise, just read until it's time to add the next odometry node.
     */
    private void processLog(boolean all)
    {
        try {
            do {
                // Use log to construct the next node for our graph and all
                // associated edges.
                lcm.logging.Log.Event event = null;
                while (true) {
                    boolean done = false;
                    event = log.readNext();
                    //if (event.channel.equals("POSE"))
                        // done |= handlePose(event);
                    if (event.channel.equals("ODOM_IMU"))
                        done |= handleIMU(event);
                    //else if (event.channel.equals("TAG_DETECTIONS_TX"))
                    //    handleTags(event);
                    else if (event.channel.equals("IMAGE"))
                        handleIm(event);
                    else if (event.channel.equals("DYNAMIXEL_STATUS_1"))
                        handleDynamixel(event);
                    else if (event.channel.equals("HOKUYO_LIDAR"))
                        handleLaser(event);

                    if (!done) {
                        continue;
                    } else {
                        counter++;
                        if (counter >= 30 && true) {
                            solver.iterate();
                            counter = 0;
                        }
                        //redraw();
                        if (!drawing) {
                            drawing = true;
                            new DrawThread().start();
                        }
                        break;
                    }
                }
            } while (all);
        } catch (IOException ex) {
            System.out.println("End of log reached.");
        }
    }

    /** Yeah. */
    private void postprocess()
    {
        // XXX
    }

    private void handleDynamixel(lcm.logging.Log.Event event) throws IOException
    {
        dynamixel_status_t status = new dynamixel_status_t(event.data);
        lastDynamixel = status;
        // dynamixelStatus.add(status);
    }

    private boolean handleIMU(lcm.logging.Log.Event event) throws IOException
    {
        odom_imu_t odom = new odom_imu_t(event.data);
        pose_t pose = odometry.getPose(odom);
        return handlePose(pose);
    }

    private boolean handlePose(lcm.logging.Log.Event event) throws IOException
    {
        pose_t pose = new pose_t(event.data);
        return handlePose(pose);
    }

    private boolean handlePose(pose_t pose) throws IOException
    {
        poses.add(pose);
        if (lastPose == null)
            lastPose = pose;
        if (pose.utime - lastPose.utime < STEP_TIME_US)
            return false;

        // Calculate RBT between last pose and now
        double[] dq = LinAlg.quatMultiply(pose.orientation,
                                          LinAlg.quatInverse(lastPose.orientation));
        // XYZ in local world coordinates. Need to transform to robot's
        // local coords.
        double[] dxyz = LinAlg.subtract(pose.pos,
                                        lastPose.pos);
        dxyz = LinAlg.quatRotate(LinAlg.quatInverse(lastPose.orientation),
                                 dxyz);
        double[][] T = LinAlg.quatPosToMatrix(dq, dxyz);

        // Create new node and edge.
        GXYTNode prevNode = (GXYTNode) g.nodes.get(poseIdxs.get(poseIdxs.size()-1));
        double[][] pT = LinAlg.xytToMatrix(prevNode.state);
        double[][] nT = LinAlg.matrixAB(pT, T);

        GXYTNode n = new GXYTNode();
        n.state = LinAlg.matrixToXYT(nT);
        n.init = LinAlg.matrixToXYT(nT);
        n.truth = null; // We don't know the truth
        n.setAttribute("type", "robot");

        // Associate the most recent laser_t with this node
        laser_t laser = laserFix.getFixedLaserT(pose);  //lasers.get(lasers.size()-1);
        ArrayList<double[]> lpts = new ArrayList<double[]>();
        for(int i=0; i<laser.nranges; i++) {
            if(laser.ranges[i] < 0)
                continue;
            double theta = laser.rad0 + (i * laser.radstep);
            double x = laser.ranges[i] * Math.cos(theta);
            double y = laser.ranges[i] * Math.sin(theta);
            lpts.add(new double[]{x, y, (byte)0});
        }
        n.setAttribute("laser", lpts);

        GXYTEdge e = new GXYTEdge();
        e.z = new double[] {dxyz[0],
                            dxyz[1],
                            MathUtil.mod2pi(LinAlg.quatToRollPitchYaw(dq)[2])};
        e.truth = null; // We don't know the truth
        e.setAttribute("type", "odom");

        // XXX Make this proportional eventually
        e.P = new double[3][3];
        e.P[0][0] = 1.0 / LinAlg.sq(Math.abs(e.z[0]) * ODOM_ERR_DIST + ODOM_ERR_DIST_FIXED);
        e.P[1][1] = 1.0 / LinAlg.sq(Math.abs(e.z[1]) * ODOM_ERR_DIST + ODOM_ERR_DIST_FIXED);
        e.P[2][2] = 1.0 / LinAlg.sq(Math.abs(e.z[2]) * ODOM_ERR_ROT + ODOM_ERR_ROT_FIXED);
        e.nodes = new int[] {poseIdxs.get(poseIdxs.size()-1), g.nodes.size()};
        poseIdxs.add(g.nodes.size());

        g.nodes.add(n);
        g.edges.add(e);
        lastPose = pose;
        return true;
    }

    private void handleIm(lcm.logging.Log.Event event) throws IOException
    {
        image_t it = new image_t(event.data);
        BufferedImage im = tagFix.getImage(it);
        tag_detection_list_t tl = tagFix.getTags(im, detector);
        tl.utime = it.utime;
        if (true) {
            double scale = 0.25;
            VisWorld.Buffer vb = vw.getBuffer("im");
            vb.setDrawOrder(-100);
            vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
                                        LinAlg.scale(scale),
                                        new VzImage(im, VzImage.FLIP)));

            double[] K = tagFix.getCameraParams();
            ArrayList<double[]> points = new ArrayList<double[]>();
            points.add(new double[] {K[2], im.getHeight()-1-K[3]});
            points.add(new double[] {0,0});
            points.add(new double[] {0,im.getHeight()});
            points.add(new double[] {im.getWidth(), 0});
            points.add(new double[] {im.getWidth(), im.getHeight()});
            vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
                                        LinAlg.scale(scale),
                                        new VzPoints(new VisVertexData(points),
                                                     new VzPoints.Style(Color.yellow, 5))));
            for (tag_detection_t td: tl.detections) {
                ArrayList<double[]> red = new ArrayList<double[]>();
                ArrayList<double[]> green = new ArrayList<double[]>();
                ArrayList<double[]> blue = new ArrayList<double[]>();
                red.add(new double[] {td.pxy[0][0], im.getHeight()-1-td.pxy[0][1]});
                red.add(new double[] {td.pxy[1][0], im.getHeight()-1-td.pxy[1][1]});
                blue.add(new double[] {td.pxy[1][0], im.getHeight()-1-td.pxy[1][1]});
                blue.add(new double[] {td.pxy[2][0], im.getHeight()-1-td.pxy[2][1]});
                blue.add(new double[] {td.pxy[2][0], im.getHeight()-1-td.pxy[2][1]});
                blue.add(new double[] {td.pxy[3][0], im.getHeight()-1-td.pxy[3][1]});
                green.add(new double[] {td.pxy[3][0], im.getHeight()-1-td.pxy[3][1]});
                green.add(new double[] {td.pxy[0][0], im.getHeight()-1-td.pxy[0][1]});
                vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
                                            LinAlg.scale(scale),
                                            new VzLines(new VisVertexData(red),
                                                        VzLines.LINES,
                                                        new VzLines.Style(Color.red, 2))));
                vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
                                            LinAlg.scale(scale),
                                            new VzLines(new VisVertexData(green),
                                                        VzLines.LINES,
                                                        new VzLines.Style(Color.green, 2))));
                vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_LEFT,
                                            LinAlg.scale(scale),
                                            new VzLines(new VisVertexData(blue),
                                                        VzLines.LINES,
                                                        new VzLines.Style(Color.blue, 2))));
            }
            vb.swap();
        }
        handleTags(tl);
    }

    private void handleTags(lcm.logging.Log.Event event) throws IOException
    {
        tag_detection_list_t tl = new tag_detection_list_t(event.data);
        handleTags(tl);
    }

    private void handleTags(tag_detection_list_t tl)
    {
        tags.add(tl);

        for (tag_detection_t td: tl.detections) {
            // Find tag pose relative to the camera
            double[][] T_h = homographyToRBT(td);

            // Filter out bad tags. Roll should be near Pi. Pitch near 0.
            double[] xyzrpy = LinAlg.matrixToXyzrpy(T_h);
            //LinAlg.print(xyzrpy);
            double thresh = Math.toRadians(15);
            double dr = Math.abs(MathUtil.mod2pi(xyzrpy[3]-Math.PI));
            double dp = Math.abs(MathUtil.mod2pi(xyzrpy[4]));
            if (dr > thresh || dp > thresh)
                continue;

            // Grab most recent pose. If not exists, ignore this tag.
            if (poses.size() < 1)
                continue;
            pose_t pose = poses.get(poses.size()-1);

            // Calculate RBT between last pose and now
            double[] dq = LinAlg.quatMultiply(pose.orientation,
                                              LinAlg.quatInverse(lastPose.orientation));
            // XYZ in local world coordinates. Need to transform to robot's
            // local coords.
            double[] dxyz = LinAlg.subtract(pose.pos,
                                            lastPose.pos);
            dxyz = LinAlg.quatRotate(LinAlg.quatInverse(lastPose.orientation),
                                     dxyz);
            double[][] T_p = LinAlg.quatPosToMatrix(dq, dxyz);

            GXYTNode prevNode = (GXYTNode) g.nodes.get(poseIdxs.get(poseIdxs.size()-1));
            double[][] pT = LinAlg.xytToMatrix(prevNode.state);
            double[][] nT = LinAlg.matrixAB(pT, T_p);
            nT = LinAlg.matrixAB(nT, T_h);

            // If this is the first time we've seen a particular tag, add a new
            // XY state node for it. Note: We may strand this node w/o edge!
            // We'll come back for edges later.
            if (!tagIdxs.containsKey(td.id)) {
                GXYTNode n = new GXYTNode();
                n.state = LinAlg.matrixToXYT(nT);
                n.init = LinAlg.matrixToXYT(nT);
                n.truth = null; // We don't know the truth
                n.setAttribute("type", "tag");
                n.setAttribute("id", new Integer(td.id));

                tagIdxs.put(td.id, g.nodes.size());
                g.nodes.add(n);
            }

            // Make an edge if applicable
            if (!tag2pose.containsKey(td.id))
                tag2pose.put(td.id, new HashSet<Integer>());
            if (!tag2pose.get(td.id).contains(poseIdxs.get(poseIdxs.size()-1))) {
                GXYTEdge e = new GXYTEdge();

                double[][] T = LinAlg.matrixAB(T_p, T_h);

                e.z = new double[] {T[0][3],
                                    T[1][3],
                                    MathUtil.mod2pi(LinAlg.matrixToRollPitchYaw(T)[2])};
                e.truth = null; // We don't know the truth.
                // You can't handle the truth!

                // XXX This will need to change
                e.P = new double[3][3];
                e.P[0][0] = 1.0 / LinAlg.sq(TAG_ERR_TRANS);
                e.P[1][1] = 1.0 / LinAlg.sq(TAG_ERR_TRANS);
                e.P[2][2] = 1.0 / LinAlg.sq(TAG_ERR_ROT);
                e.nodes = new int[] {poseIdxs.get(poseIdxs.size()-1),
                                     tagIdxs.get(td.id)};
                e.setAttribute("type", "tag");
                e.setAttribute("id", new Integer(td.id));

                tag2pose.get(td.id).add(poseIdxs.get(poseIdxs.size()-1));
                g.edges.add(e);
            }


         }
    }

    private double[][] homographyToRBT(tag_detection_t td)
    {
        double[] K = tagFix.getCameraParams();
        double[][] H = new double[3][3];
        for (int i = 0; i < 3; i++) {
            for (int j = 0; j < 3; j++) {
                H[i][j] = td.H[i][j];
            }
        }

        // Correct for 80 degree tilt XXX

        //double[][] M = CameraUtil.homographyToPose(-fx, fy, cx, cy, H);
        double[][] M = CameraUtil.homographyToPose(-K[0], K[1], K[2], K[3], H);
        M = CameraUtil.scalePose(M, 2.0, tagSize);
        //double[][] xform = LinAlg.matrixAB(LinAlg.rotateX(-Math.toRadians(170)),
        //                                   LinAlg.rotateZ(Math.PI/2));
        //M = LinAlg.matrixAB(xform, M);
        double[][] xform = new double[][] {{0, -1, 0, 0},
                                           {-1, 0, 0, 0},
                                           {0, 0, -1, 0},
                                           {0, 0,  0, 1}};
        M = LinAlg.matrixAB(xform, M);
        M = LinAlg.matrixAB(M, LinAlg.rotateZ(Math.PI));

        //System.out.println("\nTag "+td.id);
        //LinAlg.print(LinAlg.matrixToXyzrpy(M));

        if (false) {
            vw.getBuffer("debug").addBack(new VisChain(M,
                                                       LinAlg.scale(0.25),
                                                       new VzRectangle(new VzMesh.Style(Color.white))));
            vw.getBuffer("debug").swap();
        }

        return M;
    }

    private void handleLaser(lcm.logging.Log.Event event) throws IOException
    {
        // XXX TODO
        if(lastPose == null || lastDynamixel == null)
            return;

        laser_t hokuyo_laser = new laser_t(event.data);
        laserFix.addData(poses.get(poses.size()-1), hokuyo_laser, lastDynamixel);
    }

    private void redraw()
    {
        VisWorld.Buffer vbTags = vw.getBuffer("tags");
        vbTags.setDrawOrder(10);
        VisWorld.Buffer vbRobots = vw.getBuffer("poses");
        vbRobots.setDrawOrder(0);
        VisWorld.Buffer vbTagObs= vw.getBuffer("tag-obs");
        vbTagObs.setDrawOrder(-5);
        VisWorld.Buffer vbTraj = vw.getBuffer("trajectory");
        vbTraj.setDrawOrder(-10);
        VisWorld.Buffer vbLaser = vw.getBuffer("lasers");
        vbLaser.setDrawOrder(-15);
        VisWorld.Buffer vbGridMap = vw.getBuffer("grid-map");

        ArrayList<double[]> lines = new ArrayList<double[]>();
        ArrayList<double[]> taglines = new ArrayList<double[]>();

        for (GEdge o: g.edges) {
            if (o instanceof GXYTEdge) {
                GXYTEdge e = (GXYTEdge)o;

                if (e.nodes.length < 2)
                    continue;

                String type = (String)e.getAttribute("type");
                GXYTNode a = (GXYTNode) g.nodes.get(e.nodes[0]);
                GXYTNode b = (GXYTNode) g.nodes.get(e.nodes[1]);

                if (type == null) {
                    continue;
                } else if (type.equals("odom")) {
                    lines.add(LinAlg.resize(a.state, 2));
                    lines.add(LinAlg.resize(b.state, 2));
                } else if (type.equals("tag")) {
                    taglines.add(LinAlg.resize(a.state, 2));
                    taglines.add(LinAlg.resize(b.state, 2));
                }
            }
        }
        vbTraj.addBack(new VzLines(new VisVertexData(lines),
                                   VzLines.LINES,
                                   new VzLines.Style(Color.blue, 2)));
        vbTagObs.addBack(new VzLines(new VisVertexData(taglines),
                                     VzLines.LINES,
                                     new VzLines.Style(Color.magenta, 2)));

        // Draw a gridmap?
        GridMap gm = GridMap.makeMeters(-20, -5,
                                        gridmap_size, gridmap_size,
                                        metersPerPixel, 255);

        for (GNode o: g.nodes) {
            if (o instanceof GXYTNode) {
                GXYTNode n = (GXYTNode)o;
                String type = (String)n.getAttribute("type");
                if (type == null) {
                    continue;
                } else if (type.equals("robot")) {
                    vbRobots.addBack(new VisChain(LinAlg.xytToMatrix(n.state),
                                                  new VzRobot(Color.green)));

                    ArrayList<double[]> laserPts = (ArrayList<double[]>)n.getAttribute("laser");
                    if(laserPts != null) {
                        vbLaser.addBack(new VisChain(LinAlg.xytToMatrix(n.state),
                                                     new VzPoints(new VisVertexData(laserPts),
                                                                  new VzPoints.Style(Color.cyan, 1))));

                        // Add points into gridmap
                        if(DRAW_GRIDMAP) {
                            ArrayList<double[]> gpoints = LinAlg.transform(n.state, laserPts);
                            for(double[] p : gpoints) {
                                gm.setValue(p[0], p[1], (byte)0);
                            }
                        }
                    }
                } else if (type.equals("tag")) {
                    Integer id = (Integer)n.getAttribute("id");
                    String f = String.format("%d", id);
                    vbTags.addBack(new VisChain(LinAlg.xytToMatrix(n.state),
                                                new VzCircle(0.075,
                                                             new VzMesh.Style(Color.red)),
                                                LinAlg.scale(0.1),
                                                new VzText(VzText.ANCHOR.CENTER, f)));
                }
            }
        }

        if(DRAW_GRIDMAP) {
            if (gm != null) {
                BufferedImage im = gm.makeBufferedImage();
                double vertices [][] = {gm.getXY0(), {gm.getXY1()[0],gm.getXY0()[1]},
                                        gm.getXY1(), {gm.getXY0()[0],gm.getXY1()[1]}};
                double texcoords [][] = {{0,0}, {0,im.getHeight()*gm.metersPerPixel},
                                         {im.getWidth()*gm.metersPerPixel,im.getHeight()},
                                         {im.getWidth(),0}};

                vbGridMap.addBack(new VisChain(LinAlg.translate(gm.x0, gm.y0),
                                               LinAlg.scale(gm.metersPerPixel),
                                               new VzImage(new VisTexture(gm.makeBufferedImage(),
                                                                          VisTexture.NO_MIN_FILTER |
                                                                          VisTexture.NO_MAG_FILTER))));
                vbGridMap.swap();
            }
        }

        vbTags.swap();
        vbRobots.swap();
        vbTagObs.swap();
        vbTraj.swap();
        vbLaser.swap();
    }

    public GridMap getGridMap()
    {
        GridMap gm = GridMap.makeMeters(-20, -5,
                                        gridmap_size, gridmap_size,
                                        metersPerPixel, 255);
        for (GNode o: g.nodes) {
            if (o instanceof GXYTNode) {
                GXYTNode n = (GXYTNode)o;
                String type = (String)n.getAttribute("type");
                if (type == null) {
                    continue;
                }
                else if (type.equals("robot")) {
                    ArrayList<double[]> lpts = (ArrayList<double[]>)n.getAttribute("laser");
                    if(lpts != null) {
                        ArrayList<double[]> gpoints = LinAlg.transform(n.state, lpts);
                        for(double[] p : gpoints)
                            gm.setValue(p[0], p[1], (byte)0);
                    }
                }
            }
        }

        return gm;
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Configuration file for detector");
        opts.addString('\0', "cal", null, "Configuration file for calibration");

        if (!opts.parse(args)) {
            System.err.println("Options error: "+opts.getReason());
            System.exit(-1);
        }

        boolean badUsage = false;
        ArrayList<String> extraArgs = opts.getExtraArgs();
        if (extraArgs.size() < 1)
            badUsage = true;

        if (opts.getBoolean("help") || badUsage) {
            System.out.printf("Usage: java progcog.slam.OfflineSLAM [options] log-file\n");
            opts.doHelp();
            System.exit(1);
        }

        if(extraArgs.size() > 0)
            new OfflineSLAM(opts, extraArgs.get(0));
    }
}
