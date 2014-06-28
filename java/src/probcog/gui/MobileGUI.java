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

public class MobileGUI extends JFrame implements VisConsole.Listener
{
    private ProbCogSimulator simulator;

    // Periodic tasks
    PeriodicTasks tasks = new PeriodicTasks(2);

    // Message caches
    ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);

    // Vis Stuff
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;
    GraphVisEventHandler graphHandler;

    // Temporary graph stuff (where will this graph actually live so that it is
    // accessible to all who need it?
    MultiGraph<CommandNode, CommandEdge> graph = new MultiGraph<CommandNode, CommandEdge>();

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
        graphHandler = new GraphVisEventHandler(vw, graph);
        vl.addEventHandler(graphHandler);
        this.add(vc, BorderLayout.CENTER);

        VisConsole console = new VisConsole(vw, vl, vc);
        console.addListener(this);

    	// Initialize the simulator
        // CommandInterpreter ci = new CommandInterpreter();
        simulator = new ProbCogSimulator(opts, vw, vl, vc, console);

        // Parameter stuff
        pg = new ParameterGUI();
        initParameters();
        this.add(pg, BorderLayout.SOUTH);

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
        Object classyLock = new Object();
        HashMap<Integer, classifications_t> classifications =
            new HashMap<Integer, classifications_t>();

        public RenderThread()
        {
            LCM.getSingleton().subscribe("CLASSIFICATIONS", this);
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
                drawClassifications();
                drawGraph(graph);
                TimeUtil.sleep(1000/fps);
            }
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                if ("POSE".equals(channel)) {
                    pose_t pose = new pose_t(ins);
                    poseCache.put(pose, pose.utime);
                } else if ("CLASSIFICATIONS".equals(channel)) {
                    classifications_t classy = new classifications_t(ins);
                    synchronized (classyLock) {
                        classifications.put(classy.id, classy);
                    }
                }
            } catch (IOException ex) {
                System.err.println("WRN: Error receiving message on channel - "+channel);
                ex.printStackTrace();
            }
        }

        public void drawClassifications()
        {
            synchronized (classyLock) {
                pose_t pose = poseCache.get();

                VisWorld.Buffer vb = vw.getBuffer("classifications");
                if (pose == null || classifications.size() < 1) {
                    vb.swap();
                    return;
                }
                VisVertexData vvd = new VisVertexData();
                VisColorData vcd = new VisColorData();
                for (classifications_t classy: classifications.values()) {
                    double yaw = LinAlg.quatToRollPitchYaw(pose.orientation)[2];
                    double[] rel_xyz = LinAlg.transform(LinAlg.rotateZ(yaw), LinAlg.resize(classy.xyzrpy, 3));
                    double[] xyz = LinAlg.add(pose.pos, rel_xyz);
                    vvd.add(xyz);
                    vcd.add(ColorUtil.swapRedBlue(ColorUtil.seededColor(classy.name.hashCode()).getRGB()));
                }
                classifications.clear();

                vb.addBack(new VzPoints(vvd, new VzPoints.Style(vcd, 10)));
                vb.swap();
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

        public int getDispatchOrder()
        {
            return -10000;    // Highest priority
        }

        public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
        {
            // Toggle mode
            if (e.getKeyCode() == KeyEvent.VK_G)
                graphHandler.toggle();
            VisWorld.Buffer vb = vw.getBuffer("graphMode");
            if (graphHandler.isOn()) {
                vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.TOP_LEFT,
                                            LinAlg.scale(0.1),
                                            new VzText(VzText.ANCHOR.TOP_LEFT_ROUND,
                                                       "<<monospaced-128-bold,red>>Graph Mode")));
            }
            vb.swap();
            return false;
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

    // Graph loading/saving
    private void saveGraph(String filename) throws IOException
    {
        FileOutputStream fout = new FileOutputStream(filename);
        ObjectOutputStream oout = new ObjectOutputStream(fout);
        oout.writeObject(graph);
        oout.close();
        fout.close();
    }

    private void loadGraph(String filename) throws IOException
    {
        try {
            FileInputStream fin = new FileInputStream(filename);
            ObjectInputStream oin = new ObjectInputStream(fin);
            graph = (MultiGraph<CommandNode, CommandEdge>) oin.readObject();
            oin.close();
            fin.close();
        } catch (ClassNotFoundException ex) {
            ex.printStackTrace();
        }
    }

    private void initParameters()
    {
        // Is noise on by default? Assumes simulator HAS been initialization
        boolean useNoise = simulator.getWorld().config.getBoolean("simulator.sim_magic_robot.use_noise", false);
        pg.addCheckBoxes("noise", "Sensor Noise", useNoise);

        pg.addListener(new SimParameterListener());
    }

    private void drawWorld()
    {
    	VisWorld.Buffer buffer = vw.getBuffer("object-view");
        ArrayList<SimObject> objs = simulator.getWorld().objects;
        for(SimObject o : objs) {
            buffer.addBack(o.getVisObject());
        }
    	buffer.swap();
    }

    private void drawGraph(MultiGraph<CommandNode, CommandEdge> graph)
    {
        // Drawing parameters
        double STEP_SIZE = 0.5;    // [m]
        double START_ANGLE = Math.toRadians(45);
        double ANGLE_STEP = Math.toRadians(2.5);

        ArrayList<Color> colors = new ArrayList<Color>(Palette.web.listAll());
        Collections.shuffle(colors, new Random(18941));
        HashMap<String, Color> colorMap = new HashMap<String, Color>();

        VisWorld.Buffer vb = vw.getBuffer("graph");
        synchronized (graph) {
            // Render edges...this will take some doing
            for (Integer n0: graph.getNodes()) {
                for (Integer n1: graph.getNodes()) {
                    if (n0.equals(n1))
                        continue;

                    Set<CommandEdge> edges = graph.getEdges(n0, n1);
                    if (edges == null)
                        continue;

                    CommandNode a = graph.getValue(n0);
                    CommandNode b = graph.getValue(n1);
                    if (a == null || b == null)
                        continue;

                    double[] axy = a.getXY();
                    double[] bxy = b.getXY();
                    double theta0 = Math.atan2(bxy[1]-axy[1], bxy[0]-axy[0]);

                    int idx = 0;
                    for (CommandEdge e: edges) {
                        // Pick a color from the palette
                        if (!colorMap.containsKey(e.law))
                            colorMap.put(e.law, colors.get(colorMap.size()%colors.size()));
                        Color color = colorMap.get(e.law);

                        ArrayList<double[]> points = new ArrayList<double[]>();
                        points.add(axy);

                        double theta1 = MathUtil.mod2pi(theta0 + (START_ANGLE + idx*ANGLE_STEP));
                        double theta2 = MathUtil.mod2pi(Math.PI + theta0 - (START_ANGLE + idx*ANGLE_STEP));
                        double r = (idx+1)*STEP_SIZE;
                        double[] xy0 = LinAlg.add(axy, new double[] {r*Math.cos(theta1), r*Math.sin(theta1)});
                        double[] xy1 = LinAlg.add(bxy, new double[] {r*Math.cos(theta2), r*Math.sin(theta2)});
                        points.add(xy0);
                        points.add(xy1);

                        points.add(bxy);
                        vb.addBack(new VzLines(new VisVertexData(points),
                                               VzLines.LINE_STRIP,
                                               new VzLines.Style(color, 1)));
                        idx++;
                    }
                }
            }

            // Render vertices
            ArrayList<double[]> points = new ArrayList<double[]>();
            for (Integer key: graph.getNodes()) {
                CommandNode n = graph.getValue(key);
                if (n == null)
                    continue;
                points.add(n.getXY());
            }
            vb.addBack(new VzPoints(new VisVertexData(points),
                                    new VzPoints.Style(Color.red, 5)));
        }

        vb.swap();
    }

    // XXX Should be updated to not draw forever/draw noisy data
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

    // === VisConsole commands ===
	public boolean consoleCommand(VisConsole console, PrintStream out, String command)
    {
        String toks[] = command.trim().split("\\s+");
        if (toks.length == 0)
            return false;

        // Graph handling commands
        if (toks[0].equals("graph")) {
            if (toks.length < 2) {
                out.printf("usage: graph <save | load> [filename]\n");
                return true;
            }
            if (toks[1].equals("load")) {
                if (toks.length != 3) {
                    out.printf("usage: graph load <filename>\n");
                } else {
                    try {
                        loadGraph(toks[2]);
                        out.printf("graph loaded\n");
                    } catch (IOException ex) {
                        out.printf("ERR: could not load graph %s\n", toks[2]);
                    }
                }
                return true;
            } else if (toks[1].equals("save")) {
                if (toks.length == 2) {
                    try {
                        saveGraph("/tmp/probcog-graph.ser");
                        out.printf("graph saved\n");
                    } catch (IOException ex) {
                        out.printf("ERR: could not save graph to file\n");
                    }
                } else if (toks.length == 3) {
                    try {
                        saveGraph(toks[2]);
                        out.printf("graph saved\n");
                    } catch (IOException ex) {
                        out.printf("ERR: could not save graph to file\n");
                    }
                } else {
                    out.printf("usage: graph save [filename]\n");
                }
                return true;
            }
        }

        return false;
    }

    public ArrayList<String> consoleCompletions(VisConsole console, String prefix)
    {
        String cs[] = new String[] { "graph load", "graph save" };

        ArrayList<String> as = new ArrayList<String>();
        for (String s: cs)
            as.add(s);

        return as;
    }
    // ============================


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
