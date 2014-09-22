package probcog.gui;

import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
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
    boolean DEBUG = true;
    LCM lcm = LCM.getSingleton();

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

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        vl.addEventHandler(new PlanningGUIEventHandler(vw));
        this.add(vc, BorderLayout.CENTER);

        VisConsole console = new VisConsole(vw, vl, vc);
        simulator = new ProbCogSimulator(opts, vw, vl, vc, console);
        //simulator.getWorld().setRunning(false); // Stop the world here, by default

        init(); // This does things like compute a full grid map for wavefront based on the sim world

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

    private void draw()
    {
        VisWorld.Buffer vb = vw.getBuffer("tag-classes");
        vb.setDrawOrder(-1000);
        try {
            TagClassifier tc = new TagClassifier(false);
            for (SimObject so: simulator.getWorld().objects) {
                if (!(so instanceof SimAprilTag))
                    continue;
                SimAprilTag tag = (SimAprilTag)so;
                Set<String> tagClasses = tc.getClasses(tag.getID());
                String name = null;
                if (tagClasses.size() > 0)
                    name = tagClasses.iterator().next();
                else
                    continue;

                int code = name.hashCode();
                Color c = ColorUtil.seededColor(code^17132477);
                vb.addBack(new VisChain(tag.getPose(),
                                        new VzRectangle(0.7, 0.7, new VzMesh.Style(c))));
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
                                            new VzSphere(0.1, new VzMesh.Style(Color.yellow))));
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

            System.out.println("TESTING MONTE CARLO METHOD");
            ArrayList<double[]> starts = new ArrayList<double[]>();
            starts.add(LinAlg.matrixToXYT(robot.getPose()));

            MonteCarloPlanner mcp = new MonteCarloPlanner(simulator.getWorld(), gm, vw);
            ArrayList<Behavior> behaviors = mcp.plan(starts, goal);
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
                bot.setPose(robot.getPose());
                for (int i = 0; i < behaviors.size(); i++) {
                    System.out.println(behaviors.get(i));
                    bot.init(behaviors.get(i).law, behaviors.get(i).test);
                    bot.simulate();
                }

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

        // Trial parameters
        int NUM_TRIALS = 2;
        ArrayList<Integer> goalIDs = new ArrayList<Integer>();

        double[][] initialPose;
        double[] L2G;

        public DataThread()
        {
            lcm.subscribe("CONTROL_LAW_STATUS.*", this);
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                if (channel.startsWith("CONTROL_LAW_STATUS")) {
                    control_law_status_list_t status = new control_law_status_list_t(ins);
                    synchronized (statusLock) {
                        lastStatus = status;
                        statusLock.notifyAll();
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
            Random r = new Random();
            initGoals(r);

            // Find the sim robot and save pose for test reset
            SimRobot robot = getRobot();
            initialPose = robot.getPose();
            L2G = robot.getL2G();

            // First, try the wavefront follower
            System.out.println("Trying wavefront...");
            tryWavefront();

            // Reset the robot pose
            System.out.println("Resetting...");
            robot.setPose(initialPose);

            // Then, try our planner
            System.out.println("Trying Monte Carlo...");
            tryMonteCarlo();

            System.out.println("DONE!");
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
            ArrayList<Integer> hallways = new ArrayList(tc.getIDsForClass("hallway"));
            assert (hallways.size() > 1);

            int idx = r.nextInt(hallways.size());
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
        private void tryWavefront()
        {
            for (Integer id: goalIDs) {
                samePoseCount = 0;
                System.out.println("NFO: Wavefront pursuing tag "+id);
                SimRobot robot = getRobot();
                assert (robot != null);

                SimAprilTag tag = getTag(id);
                assert (tag != null);

                double[] startXY = LinAlg.matrixToXYT(robot.getNoisyPose(L2G));
                double[] goalXY = LinAlg.matrixToXYT(tag.getPose());

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

                if (path.size() < 2)
                    TimeUtil.sleep(5);

                if (DEBUG) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-wavefront-path");
                    vb.setDrawOrder(-1000);
                    vb.addBack(new VzLines(new VisVertexData(path),
                                           VzLines.LINE_STRIP,
                                           new VzLines.Style(Color.yellow, 2)));
                    vb.swap();
                }

                // Try following the path
                Tic tic = new Tic();
                Tic stepTic = new Tic();
                diff_drive_t dd = new diff_drive_t();
                while (followingPath(robot, dd)) {
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

        private void tryMonteCarlo()
        {
            MonteCarloPlanner mcp = new MonteCarloPlanner(simulator.getWorld(), gm, vw);
            ArrayList<double[]> starts = null;
            for (Integer id: goalIDs) {
                System.out.println("NFO: Wavefront pursuing tag "+id);
                SimRobot robot = getRobot();
                assert (robot != null);

                SimAprilTag tag = getTag(id);
                assert (tag != null);

                if (starts == null) {
                    starts = new ArrayList<double[]>();
                    starts.add(LinAlg.matrixToXYT(robot.getPose()));
                }

                double[] goalXY = LinAlg.matrixToXYT(tag.getPose());

                if (DEBUG) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-mc-goal");
                    vb.addBack(new VisChain(LinAlg.translate(goalXY[0], goalXY[1], 1.0),
                                            new VzSphere(0.2, new VzMesh.Style(Color.green))));
                    vb.swap();
                }

                ArrayList<Behavior> behaviors = mcp.plan(starts, goalXY, tag);

                // Here's where things get fun. We'd like to use our
                // end distribution of points represent our NEW set
                // of start positions for next time. The idea being,
                // we're trusting our simulation to keep us localized
                // enough to navigate.
                if (behaviors.size() > 0) {
                    starts = behaviors.get(behaviors.size()-1).xyts;
                } else {
                    System.out.println("ERR: Could not find a valid plan");
                    continue;
                }

                for (Behavior b: behaviors) {
                    issueCommand(b);

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
            } else {
                assert (false);
            }
            cl.utime = TimeUtil.utime();
            cl.id = commandID++;

            condition_test_t ct = null;
            if (b.test instanceof ClassificationCounterTest) {
                ct = ((ClassificationCounterTest)b.test).getLCM();
            } else if (b.test instanceof NearTag) {
                ct = ((NearTag)b.test).getLCM();
            } else {
                assert (false);
            }
            cl.termination_condition = ct;

            // Publishing
            lcm.publish("SOAR_COMMAND", cl);
            lcm.publish("SOAR_COMMAND", cl);
            lcm.publish("SOAR_COMMAND", cl);
        }


    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Global configuration file");
        opts.addString('w', "world", null, "Simulated world file");
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
