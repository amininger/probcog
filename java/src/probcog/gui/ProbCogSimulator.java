package probcog.gui;

import java.awt.*;
import java.awt.event.*;
import java.io.*;
import java.util.*;

import april.config.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

public class ProbCogSimulator implements VisConsole.Listener
{
	// Sim stuff
    SimWorld world;
    Simulator sim;

    // Vis stuff
    VisConsole console;

    public ProbCogSimulator(GetOpt opts,
                            VisWorld vw,
                            VisLayer vl,
                            VisCanvas vc)
    {
	    //vc.setTargetFPS(opts.getInt("fps"));

	    console = new VisConsole(vw, vl, vc);
	    console.addListener(this);

        loadWorld(opts);
        sim = new Simulator(vw, vl, console, world);

	}

    public SimWorld getWorld()
    {
    	return world;
    }

    private void loadWorld(GetOpt opts)
    {
    	try {
            Config config = new Config();
            //if (opts.wasSpecified("sim-config"))
            //    config = new ConfigFile(EnvUtil.expandVariables(opts.getString("sim-config")));

            if (opts.getString("world").length() > 0) {
                String worldFilePath = EnvUtil.expandVariables(opts.getString("world"));
                world = new SimWorld(worldFilePath, config);
            } else {
                world = new SimWorld(config);
            }

        } catch (IOException ex) {
            System.err.println("ERR: Error loading sim world.");
            ex.printStackTrace();
            return;
        }
    	world.setRunning(true);
    }

    // === VisConsole commands ===
    // Currently not implemented
	public boolean consoleCommand(VisConsole console, PrintStream out, String command)
    {
        return false;
    }

    public ArrayList<String> consoleCompletions(VisConsole console, String prefix)
    {
        return null;    // Only using start and stop from sim, still
    }


}
