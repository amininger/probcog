package probcog.gui;

import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import april.jmat.*;
import april.lcmtypes.*;
import april.vis.*;
import april.util.*;
import april.sim.*;

import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;
import probcog.lcmtypes.*;
import probcog.vis.*;
import probcog.util.*;
import probcog.sim.*;


public class PlanningGUI extends JFrame
{
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;
    private ProbCogSimulator simulator;

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

        this.setVisible(true);
    }

    private class PlanningGUIEventHandler extends VisEventAdapter
    {
        VisWorld world;

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

            return false;
        }
    }

    private class MonteCarloThread extends Thread
    {
        public void run()
        {
            System.out.println("TESTING MONTE CARLO METHOD");
            // Just run once for now. Initialize robot. Then simulate!
            MonteCarloBot bot = new MonteCarloBot(simulator.getWorld());
            HashMap<String, TypedValue> lawParams = new HashMap<String, TypedValue>();
            lawParams.put("side", new TypedValue((byte)-1));
            HashMap<String, TypedValue> testParams = new HashMap<String, TypedValue>();
            testParams.put("count", new TypedValue(1));
            testParams.put("class", new TypedValue("door"));

            double[] xyt = null;
            for (SimObject obj: simulator.getWorld().objects) {
                if (!(obj instanceof SimRobot))
                    continue;
                xyt = LinAlg.matrixToXYT(obj.getPose());
                break;
            }
            assert (xyt != null);
            bot.init(new FollowWall(lawParams), new ClassificationCounterTest(testParams), xyt);
            bot.simulate();

            VisWorld.Buffer vb = vw.getBuffer("test-simulation");
            vb.addBack(bot.getVisObject());
            vb.swap();
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
}
