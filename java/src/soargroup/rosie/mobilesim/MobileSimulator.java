package soargroup.rosie.mobilesim;

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
import soargroup.rosie.lcmtypes.*;

import probcog.gui.ProbCogSimulator;

// import abolt.collision.ShapeToVisObject;
// import abolt.sim.SimSensable;
// import abolt.util.SimUtil;
// import abolt.objects.SensableManager;

public class MobileSimulator extends JFrame implements VisConsole.Listener
{
    private ProbCogSimulator simulator;
    LCM lcm = LCM.getSingleton();

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

    public MobileSimulator(GetOpt opts) throws IOException
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
        HashMap<Integer, tag_classification_t> classifications =
            new HashMap<Integer, tag_classification_t>();

        public RenderThread()
        {
            lcm.subscribe("CLASSIFICATIONS", this);
            lcm.subscribe("POSE_TRUTH", this);
        }


        public void run()
        {
            Tic tic = new Tic();
            while (true) {
                double dt = tic.toctic();
                //drawWorld();
                //System.out.println(lcm.getNumSubscriptions());
                drawTrajectory(dt);
                drawClassifications();
                drawObjectLabels();
                TimeUtil.sleep(1000/fps);
            }
        }
        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                if ("POSE_TRUTH".equals(channel)) {
                    pose_t pose = new pose_t(ins);
                    poseCache.put(pose, pose.utime);
                } else if ("CLASSIFICATIONS".equals(channel)) {
                    tag_classification_list_t classy_list = new tag_classification_list_t(ins);
                    synchronized (classyLock) {
                        for (int i = 0; i < classy_list.num_classifications; i++) {
                            tag_classification_t classy = classy_list.classifications[i];
                            classifications.put(classy.tag_id, classy);
                        }
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

                // Text rendering of labels.
                HashMap<Integer, String> labels = new HashMap<Integer, String>();
                HashMap<Integer, double[]> poses = new HashMap<Integer, double[]>();

                VisWorld.Buffer vb = vw.getBuffer("classifications");
                if (pose == null || classifications.size() < 1) {
                    vb.swap();
                    vw.getBuffer("classifications-labels").swap();
                    return;
                }
                VisVertexData vvd = new VisVertexData();
                VisColorData vcd = new VisColorData();
                for (tag_classification_t classy: classifications.values()) {

                    double yaw = LinAlg.quatToRollPitchYaw(pose.orientation)[2];
                    double[] rel_xyz = LinAlg.transform(LinAlg.rotateZ(yaw), LinAlg.resize(classy.xyzrpy, 3));
                    double[] xyz = LinAlg.add(pose.pos, rel_xyz);
                    if (labels.get(classy.tag_id) == null) {
                        labels.put(classy.tag_id, "");
                        poses.put(classy.tag_id, xyz);
                    }
                    labels.put(classy.tag_id, labels.get(classy.tag_id)+classy.name+"\n");

                    vvd.add(xyz);
                    vcd.add(ColorUtil.swapRedBlue(ColorUtil.seededColor(classy.name.hashCode()).getRGB()));
                }
                classifications.clear();

                vb.addBack(new VzPoints(vvd, new VzPoints.Style(vcd, 10)));
                vb.swap();

                vb = vw.getBuffer("classifications-labels");
                for (int id: labels.keySet()) {
                    String text = "<<monospaced-128>>"+labels.get(id);
                    double[] xyz = poses.get(id);
                    vb.addBack(new VisChain(LinAlg.translate(xyz),
                                            LinAlg.scale(0.005),
                                            new VzText(VzText.ANCHOR.TOP_LEFT,
                                                       text)));
                }
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
            //if (e.getKeyCode() == KeyEvent.VK_G)
			
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
            } else if ("pp".equals(name)) {
                synchronized (simulator) {
                    for (SimObject obj: simulator.getWorld().objects) {
                        if (!(obj instanceof probcog.sim.SimRobot))
                            continue;
                        probcog.sim.SimRobot robot = (probcog.sim.SimRobot)obj;
                        robot.setPerfectPose(pg.gb(name));
                    }
                }
            }
        }
    }

    private void initParameters()
    {
        // Is noise on by default? Assumes simulator HAS been initialization
        boolean useNoise = simulator.getWorld().config.getBoolean("simulator.sim_magic_robot.use_noise", false);
        boolean perfectPose = simulator.getWorld().config.getBoolean("simulator.sim_magic_robot.perfect_pose", true);
        pg.addCheckBoxes("noise", "Sensor Noise", useNoise,
                         "pp", "Perfect Pose", perfectPose);

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

    public double[][] calcFaceCameraMatrix(){
        // === XXX THE BELOW TRIES TO RENDER TEXT OVER OBJECTS ===
    	CameraPosition camera = vl.cameraManager.getCameraTarget();
		double[] forward = LinAlg.normalize(LinAlg.subtract(camera.eye, camera.lookat));
		// Spherical coordinates
        double psi = Math.PI/2.0 - Math.asin(forward[2]);   // psi = Pi/2 - asin(z)
        double theta = Math.atan2(forward[1], forward[0]);  // theta = atan(y/x)
        if(forward[0] == 0 && forward[1] == 0){
        	theta = -Math.PI/2;
        }
        double[][] tilt = LinAlg.rotateX(psi); 				// tilt up or down to face the camera vertically
        double[][] rot = LinAlg.rotateZ(theta + Math.PI/2); // rotate flat to face the camera horizontally
        double[][] faceCamera = LinAlg.matrixAB(rot, tilt);
        return faceCamera;
    }

    private void drawObjectLabels(){
        double[][] faceCamera = calcFaceCameraMatrix();
		VisWorld.Buffer buffer = vw.getBuffer("obj-labels");
        synchronized (simulator) {
            for (SimObject obj: simulator.getWorld().objects) {
                if (!(obj instanceof probcog.sim.SimObjectPC))
                    continue;
                probcog.sim.SimObjectPC pcobj = (probcog.sim.SimObjectPC)obj;

        		String tf="<<monospaced,black,dropshadow=false>>";
        		String text = String.format("%s%s\n", tf, pcobj.getDescription());

        		VzText vzText = new VzText(text);
        		double[] textLoc = new double[]{pcobj.getXYZRPY()[0], pcobj.getXYZRPY()[1], pcobj.getXYZRPY()[2] + 1.5};
                buffer.addBack(new VisChain(LinAlg.translate(textLoc), faceCamera, LinAlg.scale(0.05), vzText));
            }
    	}
        buffer.swap();
    }


    // XXX Should be updated to not draw forever/draw noisy data
    private static final double MAX_POINTS = 500;
    private static double dtAcc = 0;
    ArrayList<double[]> poseList = new ArrayList<double[]>();
    private void drawTrajectory(double dt)
    {
        if (dtAcc + dt < .5) {
            dtAcc += dt;
            return;
        }

        pose_t pose = poseCache.get();
        if (pose != null)
            poseList.add(pose.pos);

        while (poseList.size() > MAX_POINTS)
            poseList.remove(0);

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

		// Handle tokens (toks array) and return true

        return false;
    }

    public ArrayList<String> consoleCompletions(VisConsole console, String prefix)
    {
        String cs[] = new String[] {  }; // Add any console command completions

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
        //opts.addString('c', "config", null, "Global configuration file");
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
            MobileSimulator sim = new MobileSimulator(opts);
            if(opts.getBoolean("spoof")) {
                probcog.gui.CommandSpoofer spoof = new probcog.gui.CommandSpoofer();
            }
        } catch (IOException ioex) {
            System.err.println("ERR: Error starting GUI");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}
