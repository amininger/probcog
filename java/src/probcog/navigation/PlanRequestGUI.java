package probcog.navigation;

import java.awt.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.util.*;
import april.sim.*;

import probcog.lcmtypes.*;
import probcog.sim.*;

/** A test GUI for generating plan requests. */
public class PlanRequestGUI
{
    SimWorld world = null;

    public PlanRequestGUI(GetOpt gopt) throws IOException
    {
        JFrame jf = new JFrame("Plan Request Test GUI");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());

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
        jf.add(pg, BorderLayout.CENTER);

        jf.setMinimumSize(new Dimension(400, 0));
        jf.pack();
        jf.setVisible(true);
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('w', "world", null, "Optional sim world w/tag locations");

        if (!opts.parse(args) || opts.getBoolean("help")) {
            System.err.printf("Usage: %s <options>\n", args[0]);
            opts.doHelp();
            System.exit(1);
        }

        try {
            new PlanRequestGUI(opts);
        } catch (IOException ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
        }
    }
}
