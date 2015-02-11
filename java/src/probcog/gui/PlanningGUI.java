package probcog.gui;

import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.text.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.lcmtypes.*;
import april.vis.*;
import april.util.*;
import april.sim.*;

import probcog.classify.*;
import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.lcmtypes.*;
import probcog.navigation.*;
import probcog.vis.*;
import probcog.util.*;
import probcog.sim.*;
import probcog.robot.control.*;


public class PlanningGUI extends JFrame implements LCMSubscriber
{
    int NUM_TRIALS = 100;
    boolean DEBUG = true;
    LCM lcm = LCM.getSingleton();
    GetOpt opts;

    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;
    int commandID = 0;
    private ProbCogSimulator simulator;
    private GridMap gm;
    private WavefrontPlanner wfp;
    private double[] goal = new double[2];

    ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);

    public PlanningGUI(GetOpt opts)
    {
        super("Planning GUI");
        this.setSize(800, 600);
        this.setLayout(new BorderLayout());
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.opts = opts;

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        vl.addEventHandler(new PlanningGUIEventHandler(vw));
        this.add(vc, BorderLayout.CENTER);

        VisConsole console = new VisConsole(vw, vl, vc);
        simulator = new ProbCogSimulator(opts, vw, vl, vc, console);
        //simulator.getWorld().setRunning(false); // Stop the world here, by default

        init(); // This does things like compute a full grid map for wavefront based on the sim world

        NUM_TRIALS = opts.getInt("num-trials");

        // Render some bonus information about tags types, etc.
        vl.backgroundColor = new Color(0x55, 0x55, 0x55, 0xff);
        draw();

        lcm.subscribe("POSE", this);

        this.setVisible(true);
    }

    // === GUI And Planning Functionality =====================================
    private void init()
    {
        // Pre-create a static grid map based on the simulated world for the
        // wavefront path planner to use
        if (true) {
            createGridMap();
        }
    }

    private void createGridMap()
    {
        if (simulator.getWorld().objects.size() < 1)
            return;

        // Set dimensions
        double max[] = {-Double.MAX_VALUE, - Double.MAX_VALUE,- Double.MAX_VALUE};
        double min[] = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        for (SimObject so : simulator.getWorld().objects) {
            double T[][] = so.getPose();
            april.sim.Shape s = so.getShape();
            if (s instanceof BoxShape) {
                BoxShape bs = (BoxShape) s;

                ArrayList<double[]> vertices = bs.getVertices();

                for (double vertex[] : vertices) {
                    double global_v[] = LinAlg.transform(T, vertex);

                    for (int l = 0; l < 3; l++) {
                        max[l] = Math.max(global_v[l],max[l]);
                        min[l] = Math.min(global_v[l],min[l]);
                    }
                }

            } else if (s instanceof SphereShape){
                SphereShape ss = (SphereShape) s;
                double r = ss.getRadius();
                for (int l = 0; l < 3; l++) {
                    max[l] = Math.max(T[l][3] + r, max[l]);
                    min[l] = Math.min(T[l][3] - r, min[l]);
                }

            } else {
                for (int l = 0; l < 3; l++) {
                    max[l] = Math.max(T[l][3],max[l]);
                    min[l] = Math.min(T[l][3],min[l]);
                }
                System.out.println("WRN: Unsupported shape type: "+s.getClass().getName());
            }
        }

        double MPP = 0.1;
        double[] down = new double[] {0, 0, -1};
        gm = GridMap.makeMeters(min[0], min[1], max[0]-min[0], max[1]-min[1], MPP, 255);

        // XXX There's probably a faster way to do this, but this was easy and it's
        // a one-time thing
        for (double y = min[1]; y < max[1]; y+=.5*MPP) {
            for (double x = min[0]; x < max[0]; x+=.5*MPP) {
                for (SimObject obj: simulator.getWorld().objects) {
                    if (!(obj instanceof SimBox))
                        continue;
                    if (Collisions.collisionDistance(new double[] {x, y, 100}, down, obj.getShape(), obj.getPose()) < Double.MAX_VALUE) {
                        gm.setValue(x, y, (byte)0);
                    }
                }
            }
        }

        wfp = new WavefrontPlanner(gm, 0.5);

        // Debugging
        //if (DEBUG) {
        //    VisWorld.Buffer vb = vw.getBuffer("debug-gridmap");
        //    vb.setDrawOrder(-2000);
        //    vb.addBack(new VisChain(LinAlg.translate(gm.x0, gm.y0),
        //                            LinAlg.scale(MPP),
        //                            new VzImage(new VisTexture(gm.makeBufferedImage(),
        //                                                       VisTexture.NO_MIN_FILTER |
        //                                                       VisTexture.NO_MAG_FILTER))));
        //    vb.swap();
        //}
    }

    // XXX Improve tag color rendering stuff...
    // Want to show a combination of how GOOD the tags are and WHAT they are
    private void draw()
    {
        VisWorld.Buffer vb = vw.getBuffer("tag-classes");
        vb.setDrawOrder(-1000);
        try {
            TagClassifier tc = new TagClassifier(false);
            java.util.List<Color> colors = Palette.qualitative_brewer0.listPrintFriendly();
            Set<String> tagClasses = tc.getAllClasses();
            HashMap<String, Color> classToColor = new HashMap<String, Color>();
            int idx = 3;
            for (String c: tagClasses) {
                classToColor.put(c, colors.get(idx % colors.size()));
                idx++;
            }

            //int[] map = {
            //    0x0000ff,
            //    0x00ff00};
            //int[] map = {
            //    0x777777,
            //    0x0000ff,
            //    0x00ff00};
            //int[] map = {
            //    0x0000ff,
            //    0x00ffff,
            //    0x00ff00};
            int[] map = {
                0x0000aa,
                0x0004ad,
                0x0009b0,
                0x000db3,
                0x0012b7,
                0x0016ba,
                0x001bbd,
                0x0020c0,
                0x0024c4,
                0x0029c7,
                0x002dca,
                0x0032cd,
                0x0036d1,
                0x003bd4,
                0x0040d7,
                0x0044db,
                0x0049de,
                0x004de1,
                0x0052e4,
                0x0056e8,
                0x005beb,
                0x0060ee,
                0x0064f1,
                0x0069f5,
                0x006df8,
                0x0072fb,
                0x0077ff,
                0x0088ff,
                0x0099ff,
                0x00aaff,
                0x00c6aa,
                0x00e255,
                0x00ff00};



            ColorMapper cm = new ColorMapper(map, 0, 1.0);
            cm = cm.swapRedBlue();

            for (SimObject so: simulator.getWorld().objects) {
                if (!(so instanceof SimAprilTag))
                    continue;
                SimAprilTag tag = (SimAprilTag)so;
                tagClasses = tc.getClasses(tag.getID());
                String name = null;
                if (tagClasses.size() > 0)
                    name = tagClasses.iterator().next();
                else
                    continue;
                double pct = tc.correctProbability(tag.getID());
                int alpha = ((int)(0xff * pct)) << 24;

                //Color temp = classToColor.get(name);
                //int argb = temp.getRGB();
                //argb = 0xffffff & argb | alpha;
                //Color c = new Color(argb, true);
                Color c = classToColor.get(name);
                vb.addBack(new VisLighting(false,
                                           tag.getPose(),
                                           new VzCircle(0.6,
                                                        new VzMesh.Style(new Color(cm.map(pct))),
                                                        new VzLines.Style(Color.black, 1)),
                                           LinAlg.translate(0, 0.3, 0),
                                           new VzRectangle(1.2, 0.6,
                                                           new VzMesh.Style(c),
                                                           new VzLines.Style(Color.black, 1))));
                                           //LinAlg.translate(0, -0.5, 0),
                                           //new VzRectangle(1.0, 0.5,
                                           //                new VzMesh.Style(new Color(cm.map(pct))),
                                           //                new VzLines.Style(Color.black, 1))));
            }
        } catch (IOException ex) {
            ex.printStackTrace();
        }
        vb.swap();
    }

    private SimRobot getRobot()
    {
        SimRobot robot = null;
        for (SimObject obj: simulator.getWorld().objects) {
            if (!(obj instanceof SimRobot))
                continue;
            robot = (SimRobot)obj;
            break;
        }
        return robot;
    }

    private SimAprilTag getTag(int id)
    {
        SimAprilTag tag = null;
        for (SimObject obj: simulator.getWorld().objects) {
            if (!(obj instanceof SimAprilTag))
                continue;
            SimAprilTag temp = (SimAprilTag)obj;
            if (temp.getID() == id) {
                tag = temp;
                break;
            }
        }

        return tag;
    }


    // === Support Classes ====================================================
    private class PlanningGUIEventHandler extends VisEventAdapter
    {
        VisWorld world;
        Tic clickTimer = new Tic();

        public PlanningGUIEventHandler(VisWorld vw)
        {
            this.world = vw;
        }

        public int getDispatchOrder()
        {
            return -10000;    // Highest priority
        }

        public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
        {
            // Toggle mode
            if (e.getKeyCode() == KeyEvent.VK_T)
                new MonteCarloThread().start();
            if (e.getKeyCode() == KeyEvent.VK_W)
                new WavefrontThread().start();
            if (e.getKeyCode() == KeyEvent.VK_Q)
                new DataThread().start();
            if (e.getKeyCode() == KeyEvent.VK_S)
                new SpanningTreeThread().start();
            if (e.getKeyCode() == KeyEvent.VK_A)
                new AllTreesThread().start();

            return false;
        }

        public boolean mouseClicked(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            int mods = e.getModifiersEx();
            boolean shift = (mods & MouseEvent.SHIFT_DOWN_MASK) > 0;
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) > 0;

            // Set goal
            if (shift) {
                double time = clickTimer.toctic();
                if (time < 0.4) {
                    goal = LinAlg.resize(ray.intersectPlaneXY(), 2);
                    VisWorld.Buffer vb = world.getBuffer("goal");
                    vb.addBack(new VisChain(LinAlg.translate(goal),
                                            LinAlg.scale(0.5),
                                            new VzStar(new VzMesh.Style(Color.yellow))));
                    vb.swap();
                    return true;
                }
            }

            return false;
        }
    }

    private class MonteCarloThread extends Thread
    {
        public void run()
        {
            if (DEBUG) {
                VisWorld.Buffer vb = vw.getBuffer("test-simulation");
                vb.swap();
            }

            SimRobot robot = getRobot();
            double dist = Double.MAX_VALUE;
            SimAprilTag tag = null;
            for (SimObject so: simulator.getWorld().objects) {
                if (!(so instanceof SimAprilTag))
                    continue;
                double d = LinAlg.distance(goal, LinAlg.matrixToXYT(so.getPose()), 2);
                if (d < dist) {
                    dist = d;
                    tag = (SimAprilTag)so;
                }
            }

            System.out.println("TESTING MONTE CARLO METHOD");
            ArrayList<double[]> starts = new ArrayList<double[]>();
            starts.add(LinAlg.matrixToXYT(robot.getPose()));

            MonteCarloPlanner mcp = new MonteCarloPlanner(simulator.getWorld(), gm, vw);
            ArrayList<Behavior> behaviors = mcp.plan(starts, goal, tag);
            if (behaviors.size() < 1) {
                System.err.println("ERR: Did not find a valid set of behaviors");
                return;
            }
            // Just run once for now. Initialize robot. Then simulate!
            //HashMap<String, TypedValue> lawParams = new HashMap<String, TypedValue>();
            //lawParams.put("side", new TypedValue((byte)1));
            //HashMap<String, TypedValue> testParams = new HashMap<String, TypedValue>();
            //testParams.put("count", new TypedValue(2));
            //testParams.put("class", new TypedValue("door"));

            // Visualization only
            if (DEBUG) {
                MonteCarloBot bot = new MonteCarloBot(simulator.getWorld());
                //double[] xyt = LinAlg.matrixToXYT(robot.getPose());
                do {
                    bot.setPose(robot.getPose());
                    for (int i = 0; i < behaviors.size(); i++) {
                        System.out.println(behaviors.get(i));
                        bot.init(behaviors.get(i).law, behaviors.get(i).test);
                        bot.simulate();
                    }
                } while (!bot.success());

                VisWorld.Buffer vb = vw.getBuffer("test-simulation");
                vb.setDrawOrder(-900);
                vb.addBack(bot.getVisObject());
                vb.swap();

                // Show behaviors distribution
                vb = vw.getBuffer("test-distribution");
                vb.setDrawOrder(10);
                for (Behavior b: behaviors) {
                    vb.addBack(b.getVisObject());
                }
                vb.swap();
            }
        }
    }

    private class AllTreesThread extends Thread
    {
        public void run()
        {
            // Timeout is not terribly useful, as it requires knowing a few
            // things in advance: 1) How long it takes to compute the tree
            // normally and 2) How long you need to search to ensure completeness.
            // 1 can be known, but at that point, why aren't you just paying
            // the full cost. 2 cannot be known in advance.
            HashMap<Integer, Tree<Behavior> > trees =
                TreeUtil.makeTrees(simulator.getWorld(), gm, null);//, (long)(1.0*1000000));
            //TreeUtil.hist(trees);
            BehaviorGraph graph = TreeUtil.compress(trees);

            // Test it out
            ArrayList<Behavior> testPlan = graph.navigate(3, 6 , vw);

            if (testPlan == null) {
                System.err.println("ERR: No path could be found between these nodes");
                return;
            }
            MonteCarloBot bot = new MonteCarloBot(simulator.getWorld());
            Behavior start = testPlan.get(0);
            bot.setPose(LinAlg.xytToMatrix(start.xyts.get(0).endXYT)); // XXX
            for (Behavior b: testPlan) {
                if (b.law == null)
                    continue;
                System.out.println(b);
                bot.init(b.law, b.test);
                bot.simulate(true);

                assert (bot.success()); // XXX
            }

            VisWorld.Buffer vb = vw.getBuffer("test-navigation");
            vb.setDrawOrder(-900);
            vb.addBack(bot.getVisObject());
            vb.swap();
        }
    }

    private class SpanningTreeThread extends Thread
    {
        public void run()
        {
            // Pick the tag
            double dist = Double.MAX_VALUE;
            SimAprilTag tag = null;
            for (SimObject so: simulator.getWorld().objects) {
                if (!(so instanceof SimAprilTag))
                    continue;
                double d = LinAlg.distance(goal, LinAlg.matrixToXYT(so.getPose()), 2);
                if (d < dist) {
                    dist = d;
                    tag = (SimAprilTag)so;
                }
            }

            if (tag == null) {
                System.out.println("No tag selected.");
                return;
            }

            System.out.println("Building tree from tag "+tag.getID());
            MonteCarloPlanner mcp = new MonteCarloPlanner(simulator.getWorld(),
                                                          gm,
                                                          null);

            Tree<Behavior> tree = mcp.buildSpanningTree(tag.getID());
            System.out.println("Done! Built tree size "+tree.size());

            TreeUtil.renderTree(tree, simulator.getWorld(), vw.getBuffer("spanning-tree"));
        }
    }

    private class WavefrontThread extends Thread
    {
        public void run()
        {
            // XXX Noisy pose
            SimRobot robot = null;
            for (SimObject obj: simulator.getWorld().objects) {
                if (!(obj instanceof SimRobot))
                    continue;
                robot = (SimRobot)obj;
                break;
            }
            assert (robot != null);
            double[] L2G = robot.getL2G();
            double[] start = LinAlg.resize(LinAlg.matrixToXYT(robot.getNoisyPose(L2G)), 2);

            float[] costMap = wfp.getWavefront(start, goal);

            // Render the wavefront
            BufferedImage im = new BufferedImage(gm.width, gm.height, BufferedImage.TYPE_BYTE_GRAY);
            byte[] buf = ((DataBufferByte) (im.getRaster().getDataBuffer())).getData();
            for (int i = 0; i < costMap.length; i++) {
                byte v = (byte)255;
                if (costMap[i] == Float.MAX_VALUE)
                    v = (byte)0;
                else if (costMap[i] > 0)
                    v = (byte)127;
                buf[i] = v;
            }

            if (DEBUG) {
                VisWorld.Buffer vb = vw.getBuffer("debug-wavefront");
                vb.setDrawOrder(-1001);
                vb.addBack(new VisChain(LinAlg.translate(gm.x0, gm.y0),
                                        LinAlg.scale(gm.metersPerPixel),
                                        new VzImage(new VisTexture(im,
                                                                   VisTexture.NO_MIN_FILTER |
                                                                   VisTexture.NO_MAG_FILTER))));
                vb.swap();
            }


            // Get the path
            ArrayList<double[]> path = wfp.getPath();

            if (DEBUG) {
                VisWorld.Buffer vb = vw.getBuffer("debug-wavefront-path");
                vb.setDrawOrder(-1000);
                vb.addBack(new VzLines(new VisVertexData(path),
                                       VzLines.LINE_STRIP,
                                       new VzLines.Style(Color.yellow, 2)));
                vb.swap();
            }

            // Try following the path  XXX
            Tic tic = new Tic();
            Tic stepTic = new Tic();
            diff_drive_t dd = new diff_drive_t();
            while (tic.toc() < 60) {
                // Noisy pose
                double dt = stepTic.toctic();
                double[] pos = LinAlg.resize(LinAlg.matrixToXYT(robot.getNoisyPose(L2G)), 2);
                double[] orientation = LinAlg.matrixToQuat(robot.getNoisyPose(L2G));
                dd = PathControl.getDiffDrive(pos, orientation, path, Params.makeParams(), 0.8, dt);
                dd.utime = TimeUtil.utime();
                LCM.getSingleton().publish("DIFF_DRIVE", dd);
                TimeUtil.sleep(20);
            }
            dd.utime = TimeUtil.utime();
            dd.left_enabled = dd.right_enabled = true;
            dd.left = dd.right = 0;
            LCM.getSingleton().publish("DIFF_DRIVE", dd);
        }
    }

    // Run data gathering trials. Execute a patrol mission that goes around to
    // random hallways in the building for a fixed number of goals. We try to do
    // this for both a wavefront follower AND our planner. At the end, we measure
    // deviation from our actual goal positions. (TODO: Make us drive all the way
    // up to goal, if possible. We have a constant offset issue).
    private class DataThread extends Thread implements LCMSubscriber
    {
        LCM lcm = LCM.getSingleton();

        Object statusLock = new Object();
        control_law_status_list_t lastStatus = null;
        boolean statusMessageReceived = false;

        // Trial parameters
        ArrayList<Integer> goalIDs = new ArrayList<Integer>();

        double[][] initialPose;
        double[] L2G;

        // Data collection file for trajectories (fout) and then a more specific
        // one for collecting information about planning with our algorithm
        TextStructureWriter fout;
        TextStructureWriter foutPlan;

        // Track pose for data collection
        Object poseLock = new Object();
        boolean collecting = false;
        ArrayList<pose_t> poseHistory = new ArrayList<pose_t>();

        public DataThread()
        {
            try {
                String date = (new SimpleDateFormat("yyMMdd_kkmmss")).format(new Date());
                String filename = "/tmp/monte-carlo-"+date;
                String filename2 = "/tmp/mc-plan-"+date;
                fout = new TextStructureWriter(new BufferedWriter(new FileWriter(filename)));
                foutPlan = new TextStructureWriter(new BufferedWriter(new FileWriter(filename2)));
            } catch (IOException ex) {
                System.err.println("ERR: Could not open output file");
                ex.printStackTrace();
                System.exit(1);
            }

            lcm.subscribe("CONTROL_LAW_STATUS.*", this);
            lcm.subscribe("POSE_TRUTH", this);
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                if (channel.startsWith("CONTROL_LAW_STATUS")) {
                    control_law_status_list_t status = new control_law_status_list_t(ins);
                    synchronized (statusLock) {
                        lastStatus = status;
                        statusMessageReceived = true;
                        statusLock.notifyAll();
                    }
                } else if (channel.equals("POSE_TRUTH")) {
                    pose_t pose = new pose_t(ins);
                    synchronized (poseLock) {
                        boolean ok = true;
                        long utime = TimeUtil.utime();
                        if (poseHistory.size() > 0) {
                            long poseUtime = poseHistory.get(poseHistory.size()-1).utime;
                            ok = (utime-poseUtime) >= 1*1000000;
                        }
                        if (collecting && ok) {
                            poseHistory.add(pose);
                        }
                    }
                }
            } catch (IOException ex) {
                System.err.println("ERR: Could not handle message on channel - "+channel);
                ex.printStackTrace();
            }
        }

        public void run()
        {
            System.out.println("Starting long test...");

            // Create a set of goals for our test.
            System.out.println("Initializing goals...");
            Random r = new Random(58972341);
            initGoals(r);

            // Find the sim robot and save pose for test reset
            SimRobot robot = getRobot();
            initialPose = robot.getPose();
            L2G = robot.getL2G();

            try {
                fout.writeComment("Number of patrol points to visit (trials)");
                fout.writeInt(NUM_TRIALS);
                fout.writeComment("Other data will be in the following format:");
                fout.writeComment("\tgoal point");
                fout.writeComment("\ttrajectory length n");
                fout.writeComment("\ttrajectory point 0");
                fout.writeComment("\ttrajectory point 1");
                fout.writeComment("\t...");
                fout.writeComment("\ttrajectory point n-1");

                // First, try the wavefront follower
                //fout.writeComment("Noisy Wavefront Data");
                //System.out.println("Trying noisy wavefront...");
                //tryWavefront(true);

                //// Reset the robot pose
                //System.out.println("Resetting...");
                //robot.setPose(initialPose);
                //
                // Then, try our planner
                fout.writeComment("Monte Carlo Data");
                System.out.println("Trying Monte Carlo...");
                tryMonteCarlo();

                //// Reset the robot pose
                System.out.println("Resetting...");
                robot.setPose(initialPose);

                // Then, try a perfect follower
                fout.writeComment("Perfect Wavefront Data");
                System.out.println("Trying perfect wavefront...");
                tryWavefront(false);

                System.out.println("DONE!");
                fout.close();
                foutPlan.close();
            } catch (IOException ex) {
                System.err.println("ERR: something bad in I/O happened.");
                ex.printStackTrace();
            }
        }

        private void initGoals(Random r)
        {
            // Generate a set of goals
            TagClassifier tc = null;
            try {
                tc = new TagClassifier(false);
            } catch (IOException ex) {
                ex.printStackTrace();
                System.exit(1);
            }
            ArrayList<Integer> hallways = new ArrayList(tc.getIDsForClass("goal"));
            assert (hallways.size() > 1);

            // XXX Assumes robot is already on or near tag 0
            //int idx = r.nextInt(hallways.size());
            int idx;
            for (idx = 0; idx < hallways.size(); idx++) {
                if (hallways.get(idx) != 0)
                    break;
            }
            while (goalIDs.size() < NUM_TRIALS) {
                goalIDs.add(hallways.get(idx));

                int newIdx = r.nextInt(hallways.size()-1);
                if (newIdx >= idx)
                    idx = newIdx + 1;
                else
                    idx = newIdx;
            }
        }

        double[] lastPose = null;
        int samePoseCount = 0;
        private void tryWavefront(boolean noisy) throws IOException
        {
            for (Integer id: goalIDs) {
                samePoseCount = 0;
                System.out.println("NFO: Wavefront pursuing tag "+id);
                SimRobot robot = getRobot();
                assert (robot != null);

                SimAprilTag tag = getTag(id);
                assert (tag != null);

                double[][] xform = null;
                if (noisy)
                    xform = robot.getNoisyPose(L2G);
                else
                    xform = robot.getPose();
                double[] startXY = LinAlg.matrixToXYT(xform);
                double[] goalXY = LinAlg.matrixToXYT(tag.getPose());

                // File output
                fout.writeDoubles(goalXY);

                float[] costMap = wfp.getWavefront(startXY, goalXY);
                // Render the wavefront
                BufferedImage im = new BufferedImage(gm.width, gm.height, BufferedImage.TYPE_BYTE_GRAY);
                byte[] buf = ((DataBufferByte) (im.getRaster().getDataBuffer())).getData();
                for (int i = 0; i < costMap.length; i++) {
                    byte v = (byte)255;
                    if (costMap[i] == Float.MAX_VALUE)
                        v = (byte)0;
                    else if (costMap[i] > 0)
                        v = (byte)127;
                    buf[i] = v;
                }

                if (DEBUG) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-wavefront");
                    vb.setDrawOrder(-1001);
                    vb.addBack(new VisChain(LinAlg.translate(gm.x0, gm.y0),
                                LinAlg.scale(gm.metersPerPixel),
                                new VzImage(new VisTexture(im,
                                        VisTexture.NO_MIN_FILTER |
                                        VisTexture.NO_MAG_FILTER))));
                    vb.swap();

                    vb = vw.getBuffer("debug-wavefront-goal");
                    vb.addBack(new VisChain(LinAlg.translate(goalXY[0], goalXY[1], 1.0),
                                            new VzSphere(0.2, new VzMesh.Style(Color.green))));
                    vb.swap();
                }

                // Get the path
                ArrayList<double[]> path = wfp.getPath();

                if (DEBUG) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-wavefront-path");
                    vb.setDrawOrder(-1000);
                    vb.addBack(new VzLines(new VisVertexData(path),
                                           VzLines.LINE_STRIP,
                                           new VzLines.Style(Color.yellow, 2)));
                    vb.swap();
                }

                synchronized (poseLock) {
                    collecting = true;
                }

                // Try following the path
                Tic tic = new Tic();
                Tic stepTic = new Tic();
                diff_drive_t dd = new diff_drive_t();
                while (followingPath(robot, dd)) {
                    double dt = stepTic.toctic();
                    if (noisy)
                        xform = robot.getNoisyPose(L2G);
                    else
                        xform = robot.getPose();

                    double[] pos = LinAlg.resize(LinAlg.matrixToXYT(xform), 2);
                    double[] orientation = LinAlg.matrixToQuat(xform);
                    dd = PathControl.getDiffDrive(pos, orientation, path, Params.makeParams(), 0.8, dt);
                    dd.utime = TimeUtil.utime();
                    LCM.getSingleton().publish("DIFF_DRIVE", dd);
                    TimeUtil.sleep(20);
                }
                dd.utime = TimeUtil.utime();
                dd.left_enabled = dd.right_enabled = true;
                dd.left = dd.right = 0;
                LCM.getSingleton().publish("DIFF_DRIVE", dd);

                recordTrajectory();

            }

            if (DEBUG) {
                String buffers[] = new String[] {"debug-wavefront-path",
                                                 "debug-wavefront-goal",
                                                 "debug-wavefront"};
                for (String name: buffers) {
                    VisWorld.Buffer vb = vw.getBuffer(name);
                    vb.swap();
                }
            }
        }

        private boolean followingPath(SimRobot robot, diff_drive_t dd)
        {
            // We are following the path when:
            // 1) The robot has not yet reached what (it thinks) is the goal
            // 2) The robot has not collided with anything and
            if (dd.utime > 0 && dd.left == 0 && dd.right == 0)
                return false;

            // We detect collisions by seeing if the robot has moved recently
            if (lastPose != null) {
                double[] currPose = LinAlg.matrixToXYT(robot.getPose());
                if (MathUtil.doubleEquals(lastPose[0], currPose[0]) &&
                    MathUtil.doubleEquals(lastPose[1], currPose[1]) &&
                    MathUtil.doubleEquals(lastPose[2], currPose[2]))
                {
                    samePoseCount++;
                } else {
                    samePoseCount = 0;
                }
                lastPose = currPose;
            } else {
                lastPose = LinAlg.matrixToXYT(robot.getPose());
            }

            if (samePoseCount >= 10)
                return false;

            return true;
        }

        private void tryMonteCarlo() throws IOException
        {
            foutPlan.writeComment("The number of planning trials");
            foutPlan.writeInt(NUM_TRIALS);

            foutPlan.writeComment("Trial data is recorded as follows:");
            foutPlan.writeComment("\tgoal XY");
            foutPlan.writeComment("\tplanning time");
            foutPlan.writeComment("\tnumber of plan steps m");
            foutPlan.writeComment("\t\tnumber of xyts n for step 0");
            foutPlan.writeComment("\t\txyt 0");
            foutPlan.writeComment("\t\txyt 1");
            foutPlan.writeComment("\t\t...");
            foutPlan.writeComment("\t\txyt n-1");


            MonteCarloPlanner mcp = new MonteCarloPlanner(simulator.getWorld(), gm, opts.getBoolean("vis") ? vw : null);
            ArrayList<double[]> starts = null;
            int count = 0;
            for (Integer id: goalIDs) {
                System.out.println("NFO: MonteCarlo pursuing tag "+id);
                System.out.println("NFO: This is run "+count);
                count++;

                SimRobot robot = getRobot();
                assert (robot != null);

                SimAprilTag tag = getTag(id);
                assert (tag != null);

                if (starts == null) {
                    starts = new ArrayList<double[]>();
                    starts.add(LinAlg.matrixToXYT(robot.getPose()));
                    System.out.println("NFO: Init starts");
                }

                double[] goalXY = LinAlg.matrixToXYT(tag.getPose());

                // File output
                fout.writeDoubles(goalXY);
                foutPlan.writeDoubles(goalXY);

                if (DEBUG) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-mc-goal");
                    vb.addBack(new VisChain(LinAlg.translate(goalXY[0], goalXY[1], 1.0),
                                            new VzSphere(0.2, new VzMesh.Style(Color.green))));
                    vb.swap();
                }

                // We would also probably like to record this data.
                Tic planTic = new Tic();
                ArrayList<Behavior> behaviors = mcp.plan(starts, goalXY, tag);
                double planTime = planTic.toc();
                foutPlan.writeDouble(planTime); // In seconds
                foutPlan.writeInt(behaviors.size());
                for (int i = 0; i < behaviors.size(); i++) {
                    Behavior b = behaviors.get(i);
                    foutPlan.writeInt(b.xyts.size());
                    for (int j = 0; j < b.xyts.size(); j++) {
                        foutPlan.writeDoubles(b.xyts.get(j).endXYT);
                    }
                }
                fout.flush();

                // Here's where things get fun. We'd like to use our
                // end distribution of points represent our NEW set
                // of start positions for next time. The idea being,
                // we're trusting our simulation to keep us localized
                // enough to navigate.
                if (behaviors.size() > 0) {
                    //starts = behaviors.get(behaviors.size()-1).xyts;
                    starts = new ArrayList<double[]>();
                    Behavior last = behaviors.get(behaviors.size()-1);
                    for (Behavior.XYTPair xyt: last.xyts) {
                        starts.add(LinAlg.copy(xyt.endXYT));
                    }
                } else {
                    System.out.println("ERR: Could not find a valid plan");
                }

                synchronized (poseLock) {
                    collecting = true;
                }

                for (Behavior b: behaviors) {
                    synchronized (statusLock) {
                        statusMessageReceived = false;
                        do {
                            issueCommand(b);
                            try {
                                statusLock.wait(100);
                            } catch (InterruptedException ex) {}
                        } while (!statusMessageReceived);
                        commandID++;
                    }

                    while (true) {
                        synchronized (statusLock) {
                            try {
                                statusLock.wait();
                            } catch (InterruptedException ex){}
                            if (lastStatus == null)
                                continue;

                            control_law_status_t s = null;
                            for (int i = 0; i < lastStatus.nstatuses; i++) {
                                if (lastStatus.statuses[i].id == commandID-1)
                                    s = lastStatus.statuses[i];
                            }
                            if (s != null && s.status.equals("SUCCESS")) {
                                System.out.println("WE DID IT! Moving on...");
                                break;  // XXX HANDLE ME
                            } else if (s != null && s.status.equals("FAILURE")) {
                                System.out.println("YOU SUCK!");
                                break;  // XXX HANDLE ME
                            }
                        }
                    }
                }

                recordTrajectory();

                // Fast simulation hack XXX
                /*MonteCarloBot bot = new MonteCarloBot(simulator.getWorld());
                //double[] xyt = LinAlg.matrixToXYT(robot.getPose());
                bot.setPose(robot.getPose());
                for (int i = 0; i < behaviors.size(); i++) {
                    System.out.println(behaviors.get(i));
                    bot.init(behaviors.get(i).law, behaviors.get(i).test);
                    bot.simulate(300.0);
                }
                if (true) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-failure");
                    vb.addBack(bot.getVisObject());
                    vb.swap();
                }

                ArrayList<double[]> xys = bot.getTrajectoryTruth();
                if (!bot.success()) {
                    System.out.println("FAILURE");
                }

                fout.writeInt(xys.size());
                for (double[] xy: xys)
                    fout.writeDoubles(xy);
                fout.flush();

                robot.setPose(bot.getPose());*/

            }

            if (DEBUG) {
                String buffers[] = new String[] {"debug-mc-goal"};
                for (String name: buffers) {
                    VisWorld.Buffer vb = vw.getBuffer(name);
                    vb.swap();
                }
            }
        }

        private void issueCommand(Behavior b)
        {
            // Issue the appropriate command
            control_law_t cl = null;
            if (b.law instanceof FollowWall) {
                cl = ((FollowWall)b.law).getLCM();
            } else if (b.law instanceof DriveTowardsTag) {
                cl = ((DriveTowardsTag)b.law).getLCM();
            } else if (b.law instanceof Turn) {
                cl = ((Turn)b.law).getLCM();
            } else {
                assert (false);
            }
            cl.utime = TimeUtil.utime();
            cl.id = commandID;

            condition_test_t ct = null;
            if (b.test instanceof ClassificationCounterTest) {
                ct = ((ClassificationCounterTest)b.test).getLCM();
            } else if (b.test instanceof NearTag) {
                ct = ((NearTag)b.test).getLCM();
            } else if (b.test instanceof RotationTest) {
                ct = ((RotationTest)b.test).getLCM();
            } else {
                assert (false);
            }
            cl.termination_condition = ct;

            // Publishing
            lcm.publish("SOAR_COMMAND", cl);
            lcm.publish("SOAR_COMMAND", cl);
            lcm.publish("SOAR_COMMAND", cl);
        }

        private void recordTrajectory() throws IOException
        {
            // Write out results to file
            synchronized (poseLock) {
                collecting = false;
                ArrayList<double[]> xys = new ArrayList<double[]>();
                for (pose_t pose: poseHistory) {
                    double[][] M = LinAlg.quatPosToMatrix(pose.orientation,
                                                          pose.pos);
                    xys.add(LinAlg.resize(LinAlg.matrixToXYT(M), 2));
                }
                xys.add(LinAlg.resize(LinAlg.matrixToXYT(getRobot().getPose()), 2));
                fout.writeInt(xys.size());
                for (double[] xy: xys)
                    fout.writeDoubles(xy);
                fout.flush();

                poseHistory.clear();
            }
        }


    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Global configuration file");
        opts.addString('w', "world", null, "Simulated world file");
        opts.addInt('n', "num-trials", 100, "Number of trials");
        opts.addBoolean('\0', "vis", false, "Use vis");
        //opts.addString('g', "graph", null, "Graph file");
        //opts.addBoolean('s', "spoof", false, "Open small GUI to spoof soar commands");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }

        // Spin up the GUI
        try {
            PlanningGUI gui = new PlanningGUI(opts);
            //if(opts.getBoolean("spoof")) {
            //    probcog.gui.CommandSpoofer spoof = new probcog.gui.CommandSpoofer();
            //}
        } catch (Exception ex) {
            System.err.println("ERR: Error starting GUI");
            ex.printStackTrace();
            System.exit(1);
        }
    }

    // === LCM Handling =======================================================
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if ("POSE".equals(channel)) {
                pose_t pose = new pose_t(ins);
                synchronized (poseCache) {
                    poseCache.put(pose, pose.utime);
                }
            }
        } catch (IOException ex) {
            ex.printStackTrace();
            System.exit(1);
        }
    }
}
