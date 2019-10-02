package probcog.gui;

import java.awt.*;
import java.io.*;
import javax.swing.*;
import java.util.*;

import april.util.*;
import april.vis.*;

import soargroup.rosie.mobilesim.commands.*;
import soargroup.rosie.mobilesim.commands.controls.*;

/** A tool for testing control laws in simulation. */
public class ControlPlanningGUI
{
    // World rendering
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);

    // Panels and system interaction
    ControlPlanPanel controlsPanel;

    public ControlPlanningGUI(GetOpt opts)
    {
        JFrame jf = new JFrame("Control Planning GUI");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(1200,800);

        jf.add(vc, BorderLayout.CENTER);

        // XXX Implement me
        controlsPanel = new ControlPlanPanel();
        jf.add(controlsPanel, BorderLayout.EAST);

        // jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }

    public static void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h',"help",false,"Show this help screen");

        if (!opts.parse(args)) {
            System.err.println("ERR: Option parsing - "+opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(-1);
        }

        new ControlPlanningGUI(opts);
    }
}
