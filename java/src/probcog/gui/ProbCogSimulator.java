package probcog.gui;

import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

import probcog.sim.ISimEffector;
import probcog.sim.SimObjectPC;
import april.config.Config;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.sim.Simulator;
import april.util.EnvUtil;
import april.util.GetOpt;
import april.vis.VisCanvas;
import april.vis.VisConsole;
import april.vis.VisLayer;
import april.vis.VisWorld;

public class ProbCogSimulator implements VisConsole.Listener
{
	// Sim stuff
    SimWorld world;
    Simulator sim;

    // Vis stuff
    VisConsole console;

    private Timer simulateDynamicsTimer;
    private static final int DYNAMICS_RATE = 30; // FPS to simulate dynamics at

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
        
	    simulateDynamicsTimer = new Timer();
	    simulateDynamicsTimer.schedule(new SimulateDynamicsTask(), 1000, 1000/DYNAMICS_RATE);

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

    class SimulateDynamicsTask extends TimerTask
    {
		@Override
		public void run() {
//			for(SimObject obj : world.objects){
//				if(!(obj instanceof ISimEffector)){
//					continue;
//				}
//				for(SimObject target : world.objects){
//					if(target == obj || !(target instanceof SimObjectPC)){
//						continue;
//					}
//					((ISimEffector)obj).checkObject((SimObjectPC)target);
//				}
//			}
			
		}
    }

}
