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
import probcog.commands.CommandSpoofer;
import probcog.lcmtypes.*;
import probcog.perception.*;
import probcog.sensor.*;
import probcog.sim.SimLocation;
import probcog.sim.SimObjectPC;
import probcog.util.*;
import probcog.vis.*;

// import abolt.collision.ShapeToVisObject;
// import abolt.sim.SimSensable;
// import abolt.util.SimUtil;
// import abolt.objects.SensableManager;

public class MobileGUI extends JFrame
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

    public MobileGUI(GetOpt opts) throws IOException
    {
        super("ProbCog Mobile");
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setLayout(new BorderLayout());

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        this.add(vc, BorderLayout.CENTER);

    	// Initialize the simulator
        // CommandInterpreter ci = new CommandInterpreter();
        simulator = new ProbCogSimulator(opts, vw, vl, vc);

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
