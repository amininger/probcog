package probcog.gui;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.*;
import javax.swing.*;
import java.text.*;
import java.util.*;
import java.util.Timer;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.lcmtypes.*;
import april.sim.*;
import april.util.*;
import april.vis.*;
import april.vis.VisCameraManager.CameraPosition;

import probcog.arm.*;
import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;
import probcog.commands.*;
import probcog.lcmtypes.*;
import probcog.perception.*;
import probcog.sensor.*;
import probcog.sim.*;
import probcog.util.*;
import probcog.vis.*;

// import abolt.collision.ShapeToVisObject;
// import abolt.sim.SimSensable;
// import abolt.util.SimUtil;
// import abolt.objects.SensableManager;

public class MobileGUI extends JFrame
{
    private ProbCogSimulator simulator;
    // XXX Temporary home
    //private SimpleGraph<CommandNode, CommandEdge> graph = new SimpleGraph<CommandNode, CommandEdge>();

    // Periodic tasks
    PeriodicTasks tasks = new PeriodicTasks(2);

    // Message caches
    ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);

    // Vis Stuff
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    // Parameter Stuff
    ParameterGUI pg;

    public MobileGUI(GetOpt opts) throws IOException
    {
        super("ProbCog Mobile");
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setLayout(new BorderLayout());

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        vl.addEventHandler(new MobileGUIEventHandler(vw));
        this.add(vc, BorderLayout.CENTER);

    	// Initialize the simulator
        // CommandInterpreter ci = new CommandInterpreter();
        simulator = new ProbCogSimulator(opts, vw, vl, vc);

        // Parameter stuff
        pg = new ParameterGUI();
        initParameters();
        this.add(pg, BorderLayout.SOUTH);

        // Initialize the graph. Assumes CSE Sim world, as it is hardcoded for it
        // XXX
        //initGraph();
        //instructRobot();    // XXX temporary

        // Set GUI modes
        this.setVisible(true);

        // Render updates about the world
        RenderThread rt = new RenderThread();
        rt.start();
    }

    /** Render ProbCog-specific content. */
    class RenderThread extends Thread implements LCMSubscriber
    {
        int fps = 20;

        public RenderThread()
        {
            LCM.getSingleton().subscribe("POSE", this);
        }


        public void run()
        {
            Tic tic = new Tic();
            //drawGraph();
            while (true) {
                double dt = tic.toctic();
                //drawWorld();
                drawTrajectory(dt);
                TimeUtil.sleep(1000/fps);
            }
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                if ("POSE".equals(channel)) {
                    pose_t pose = new pose_t(ins);
                    poseCache.put(pose, pose.utime);
                }
            } catch (IOException ex) {
                System.err.println("WRN: Error receiving message on channel - "+channel);
                ex.printStackTrace();
            }
        }
    }

    class MobileGUIEventHandler extends VisEventAdapter
    {
        VisWorld world;

        public MobileGUIEventHandler(VisWorld vw)
        {
            world = vw;
        }

        public boolean mouseMoved(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] xy = ray.intersectPlaneXY();
            Formatter f = new Formatter();
            f.format("<<monospaced-128>>(%.2f, %.2f)", xy[0], xy[1]);
            VisWorld.Buffer vb = world.getBuffer("coordinates");

            vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_RIGHT,
                                        LinAlg.scale(0.1),
                                        new VzText(VzText.ANCHOR.BOTTOM_RIGHT_ROUND,
                                                   f.toString())));
            vb.swap();
            return false;
        }
    }

    class SimParameterListener implements ParameterListener
    {
        public void parameterChanged(ParameterGUI pg, String name)
        {
            if ("noise".equals(name)) {
                // Look for SimRobot and turn off/on noise
                synchronized (simulator) {
                    for (SimObject obj: simulator.getWorld().objects) {
                        if (!(obj instanceof probcog.sim.SimRobot))
                            continue;
                        probcog.sim.SimRobot robot = (probcog.sim.SimRobot)obj;
                        robot.setNoise(pg.gb(name));
                    }
                }
            }
        }
    }

    private void initParameters()
    {
        // Is noise on by default? Assumes simulator HAS been initialization
        boolean useNoise = simulator.getWorld().config.getBoolean("simulator.sim_magic_robot.use_noise", false);
        pg.addCheckBoxes("noise", "Sensor Noise", useNoise);

        pg.addListener(new SimParameterListener());
    }

    // XXX A hand-coded version of the CSE graph, to get off the ground with.
    /*private void initGraph()
    {
        ArrayList<SimpleGraphNode> nodes = new ArrayList<SimpleGraphNode>();
        CommandNode n;
        CommandEdge e;
        CommandEdge.Edge v;
        n = new CommandNode(new double[]{1,0}); // In front of april-office door
        nodes.add(graph.addNode(n));
        n = new CommandNode(new double[]{12.8, 0}); // In front of soar-office door
        nodes.add(graph.addNode(n));
        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");   // count = int, string class
        v.addLawParam("side", new TypedValue((byte)-1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        v = new CommandEdge.Edge("follow-heading", "count");
        v.addLawParam("heading", new TypedValue(0.4));
        v.addLawParam("distance", new TypedValue(0.5));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        e.addEdge(v);
        v.probability = .95;
        graph.connect(nodes.get(0), nodes.get(1), e);

        n = new CommandNode(new double[]{66, -40}); // John's office
        nodes.add(graph.addNode(n));
        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)-1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        v = new CommandEdge.Edge("follow-heading", "count");
        v.addLawParam("heading", new TypedValue(-0.7));
        v.addLawParam("distance", new TypedValue(0.5));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        graph.connect(nodes.get(0), nodes.get(2), e); // april to john's

        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)-1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        v = new CommandEdge.Edge("follow-heading", "count");
        v.addLawParam("heading", new TypedValue(-0.7));
        v.addLawParam("distance", new TypedValue(0.5));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        graph.connect(nodes.get(1), nodes.get(2), e);   // soar to john's

        n = new CommandNode(new double[]{66, -24.5});   // ben's office
        nodes.add(graph.addNode(n));
        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)-1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(3));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95*.95*.95;
        e.addEdge(v);
        v = new CommandEdge.Edge("follow-heading", "count");
        v.addLawParam("heading", new TypedValue(0.7));
        v.addLawParam("distance", new TypedValue(0.5));
        v.addTermParam("count", new TypedValue(3));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95*.95*.95;
        e.addEdge(v);
        graph.connect(nodes.get(2), nodes.get(3), e);   // john's to ben's

        n = new CommandNode(new double[]{66, -21.8});   // ed's
        nodes.add(graph.addNode(n));
        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)-1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        v = new CommandEdge.Edge("follow-heading", "count");
        v.addLawParam("heading", new TypedValue(0.7));
        v.addLawParam("distance", new TypedValue(0.5));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        graph.connect(nodes.get(3), nodes.get(4), e);   // ben's to ed's

        n = new CommandNode(new double[]{66, -4.5});   // conference 1
        nodes.add(graph.addNode(n));
        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)-1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(2));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95*.95;
        e.addEdge(v);
        v = new CommandEdge.Edge("follow-heading", "count");
        v.addLawParam("heading", new TypedValue(0.7));
        v.addLawParam("distance", new TypedValue(0.5));
        v.addTermParam("count", new TypedValue(2));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95*.95;
        e.addEdge(v);
        graph.connect(nodes.get(4), nodes.get(5), e);   // ed's to conference

        n = new CommandNode(new double[]{66, -0.7});  // Kitchen
        nodes.add(graph.addNode(n));
        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)-1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        v = new CommandEdge.Edge("follow-heading", "count");
        v.addLawParam("heading", new TypedValue(0.7));
        v.addLawParam("distance", new TypedValue(0.5));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        graph.connect(nodes.get(5), nodes.get(6), e);   // conference to kitchen

        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(2));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95*.95;
        e.addEdge(v);
        graph.connect(nodes.get(0), nodes.get(6), e);   // april to kitchen

        e = new CommandEdge();
        v = new CommandEdge.Edge("follow-wall", "count");
        v.addLawParam("side", new TypedValue((byte)1));
        v.addLawParam("heading", new TypedValue(0.0));
        v.addLawParam("distance", new TypedValue(1.0));
        v.addTermParam("count", new TypedValue(1));
        v.addTermParam("class", new TypedValue("door"));
        v.probability = .95;
        e.addEdge(v);
        graph.connect(nodes.get(1), nodes.get(6), e);   // soar to kitchen
    }

    public void instructRobot()
    {
        ArrayList<CommandEdge.Edge> path = GraphUtil.bestPath(graph, new double[] {0,0}, new double[] {66, -0.7});
        for (CommandEdge.Edge edge: path) {
            System.out.println(edge);
        }

    }*/

    private void drawWorld()
    {
    	VisWorld.Buffer buffer = vw.getBuffer("object-view");
        ArrayList<SimObject> objs = simulator.getWorld().objects;
        for(SimObject o : objs) {
            buffer.addBack(o.getVisObject());
        }
    	buffer.swap();
    }

    private static double dtAcc = 0;
    ArrayList<double[]> poseList = new ArrayList<double[]>();
    private void drawTrajectory(double dt)
    {
        if (dtAcc + dt < 1.0) {
            dtAcc += dt;
            return;
        }

        pose_t pose = poseCache.get();
        if (pose != null)
            poseList.add(pose.pos);

        vw.getBuffer("trajectory").addBack(new VzPoints(new VisVertexData(poseList),
                                                        new VzPoints.Style(Color.cyan, 4)));
        vw.getBuffer("trajectory").addBack(new VzLines(new VisVertexData(poseList),
                                                       VzLines.LINE_STRIP,
                                                       new VzLines.Style(Color.blue, 1)));
        vw.getBuffer("trajectory").swap();
        dtAcc = 0;
    }

    /*private void drawGraph()
    {
        VisWorld.Buffer vb = vw.getBuffer("graph");

        ArrayList<double[]> points = new ArrayList<double[]>();
        for (SimpleGraphNode n: graph.getNodes()) {
            for (SimpleGraphNode n_: graph.neighbors(n)) {
                points.add(graph.getNodeValue(n).getXY());
                points.add(graph.getNodeValue(n_).getXY());
            }
        }

        VisVertexData vvd = new VisVertexData(points);
        vb.addBack(new VzPoints(vvd,
                                new VzPoints.Style(Color.yellow, 10)));
        vb.addBack(new VzLines(vvd,
                               VzLines.LINES,
                               new VzLines.Style(Color.red, 2)));
        vb.swap();
    }*/

    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Global configuration file");
        opts.addString('w', "world", null, "Simulated world file");
        opts.addBoolean('s', "spoof", false, "Open small GUI to spoof soar commands");

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
            MobileGUI gui = new MobileGUI(opts);
            if(opts.getBoolean("spoof")) {
                CommandSpoofer spoof = new CommandSpoofer();
            }
        } catch (IOException ioex) {
            System.err.println("ERR: Error starting GUI");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}
