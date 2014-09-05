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

import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;
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
        simulator.getWorld().setRunning(false); // Stop the world here, by default

        init(); // This does things like compute a full grid map for wavefront based on the sim world

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

        wfp = new WavefrontPlanner(gm, 0.4);

        // Debugging
        if (DEBUG) {
            VisWorld.Buffer vb = vw.getBuffer("debug-gridmap");
            vb.setDrawOrder(-2000);
            vb.addBack(new VisChain(LinAlg.translate(gm.x0, gm.y0),
                                    LinAlg.scale(MPP),
                                    new VzImage(new VisTexture(gm.makeBufferedImage(),
                                                               VisTexture.NO_MIN_FILTER |
                                                               VisTexture.NO_MAG_FILTER))));
            vb.swap();
        }
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

            System.out.println("TESTING MONTE CARLO METHOD");
            MonteCarloPlanner mcp = new MonteCarloPlanner(simulator.getWorld(), gm, vw);
            ArrayList<Behavior> behaviors = mcp.plan(goal);
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
                SimRobot robot = null;
                for (SimObject obj: simulator.getWorld().objects) {
                    if (!(obj instanceof SimRobot))
                        continue;
                    robot = (SimRobot)obj;
                    break;
                }
                assert (robot != null);
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
                vb.setDrawOrder(-901);
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
            //double[] start = LinAlg.resize(LinAlg.matrixToXYT(robot.getPose()), 2);
            double[] start = LinAlg.resize(LinAlg.matrixToXYT(getNoisyPose()), 2);

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
            diff_drive_t dd = new diff_drive_t();
            while (tic.toc() < 30) {
                // Noisy pose
                //double[] pos = LinAlg.resize(LinAlg.matrixToXYT(robot.getPose()), 2);
                //double[] orientation = LinAlg.matrixToQuat(robot.getPose());
                double[] pos = LinAlg.resize(LinAlg.matrixToXYT(getNoisyPose()), 2);
                double[] orientation = LinAlg.matrixToQuat(getNoisyPose());
                dd = PathControl.getDiffDrive(pos, orientation, path, Params.makeParams(), 0.8);
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

    public double[][] getNoisyPose()
    {
        pose_t pose = poseCache.get();
        if (pose == null) {
            System.err.println("ERR: Could not find a recent enough pose");
            return LinAlg.identity(4);
        }
        return LinAlg.quatPosToMatrix(pose.orientation, pose.pos);
    }
}
