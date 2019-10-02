package probcog.navigation;

import java.awt.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.util.*;
import april.sim.*;

import soargroup.mobilesim.lcmtypes.*;
import soargroup.mobilesim.sim.*;

/** A test GUI for generating plan requests. */
public class PlanRequestGUI
{
    static final int REQUEST_HZ = 5;

    LCM lcm = LCM.getSingleton();
    String requestChannel;
    Object requestLock = new Object();
    plan_request_t request = null;

    SimWorld world = null;

    private class LCMThread extends Thread
    {
        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/REQUEST_HZ);

                if (request == null)
                    continue;
                request.utime = TimeUtil.utime();
                lcm.publish(requestChannel, request);
            }
        }
    }

    private class PGHandler implements ParameterListener
    {
        byte id = 0;
        double[] xyt = new double[3];

        public void parameterChanged(ParameterGUI pg, String name)
        {
            if (world != null && name.equals("tag")) {
                for (SimObject so: world.objects) {
                    if (!(so instanceof SimAprilTag))
                        continue;
                    SimAprilTag tag = (SimAprilTag)so;
                    if (tag.getID() != pg.gi("tag"))
                        continue;
                    xyt = LinAlg.matrixToXYT(tag.getPose());
                }
            } else if (name.equals("x") || name.equals("y")) {
                xyt[0] = pg.gd("x");
                xyt[1] = pg.gd("y");
                xyt[2] = 0;
            } else if (name.equals("send")) {
                // Issue a plan request
                synchronized (requestLock) {
                    request = new plan_request_t();
                    request.id = id++;
                    request.goalXYT = LinAlg.copy(xyt);
                }
            }
        }
    }

    public PlanRequestGUI(GetOpt gopt) throws IOException
    {
        JFrame jf = new JFrame("Plan Request Test GUI");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());

        requestChannel = gopt.getString("request-channel");

        if (gopt.wasSpecified("world")) {
            Config config = new Config();
            world = new SimWorld(gopt.getString("world"), config);
            world.setRunning(false);
        }

        ParameterGUI pg = new ParameterGUI();
        pg.addDouble("x", "X", 0);
        pg.addDouble("y", "Y", 0);
        if (world != null) {
            pg.addInt("tag", "Tag #", 0);
        }
        pg.addButtons("send", "Send");
        pg.addListener(new PGHandler());
        jf.add(pg, BorderLayout.CENTER);

        new LCMThread().start();

        jf.setMinimumSize(new Dimension(400, 0));
        jf.pack();
        jf.setVisible(true);
    }

    static public void main(String[] args)
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help screen");
        gopt.addString('w', "world", null, "Optional sim world w/tag locations");
        gopt.addString('\0', "request-channel", "PLAN_REQUEST", "Plan request LCM channel");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            System.err.printf("Usage: %s <options>\n", args[0]);
            gopt.doHelp();
            System.exit(1);
        }

        try {
            new PlanRequestGUI(gopt);
        } catch (IOException ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }
    }
}
