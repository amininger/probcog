package soargroup.mobilesim;

import java.io.IOException;
import java.io.PrintStream;
import java.util.ArrayList;
import java.util.Timer;
import java.util.TimerTask;

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

import soargroup.mobilesim.sim.*;

// LCM Types
import lcm.lcm.*;
import soargroup.mobilesim.lcmtypes.control_law_t;

public class MobileSimulator implements VisConsole.Listener, LCMSubscriber
{
	// Sim stuff
    SimWorld world;
    Simulator sim;

    // Vis stuff
    VisConsole console;

    private Timer simulateDynamicsTimer;
    private static final int DYNAMICS_RATE = 30; // FPS to simulate dynamics at

	SimRobot robot = null;

    public MobileSimulator(GetOpt opts,
                            VisWorld vw,
                            VisLayer vl,
                            VisCanvas vc,
                            VisConsole console)
    {
	    this.console = console;//new VisConsole(vw, vl, vc);
	    this.console.addListener(this);

        loadWorld(opts);
        sim = new Simulator(vw, vl, console, world);

	    simulateDynamicsTimer = new Timer();
	    simulateDynamicsTimer.schedule(new SimulateDynamicsTask(), 1000, 1000/DYNAMICS_RATE);

        ArrayList<SimObject> simObjects;
		synchronized(world.objects){
			simObjects = (ArrayList<SimObject>)world.objects.clone();
		}
		for(SimObject obj : simObjects){
			if(obj instanceof SimRobot){
				robot = (SimRobot)obj;
			}
		}
		if(robot == null){
			System.err.println("WARNING: No SimRobot defined in the world file");
		}

		LCM.getSingleton().subscribe("SOAR_COMMAND.*", this);
	}

    public SimWorld getWorld()
    {
    	return world;
    }

	private RosieSimObject getSimObject(Integer id){
        ArrayList<SimObject> simObjects;
		synchronized(world.objects){
			simObjects = (ArrayList<SimObject>)world.objects.clone();
		}
		for(SimObject obj : simObjects){
			if(obj instanceof RosieSimObject){
				RosieSimObject simObj = (RosieSimObject)obj;
				if(simObj.getID().equals(id)){
					return simObj;
				}
			}
		}
		return null;
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

	/*************************************
	 * Handling Simulated SOAR COMMANDS
	 ************************************/

	@Override
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
        try {
        	if (channel.startsWith("SOAR_COMMAND") && !channel.startsWith("SOAR_COMMAND_STATUS")){
		  		control_law_t controlLaw = new control_law_t(ins);
				if(controlLaw.name.equals("pick-up")){
					handlePickUpCommand(controlLaw);
				} else if(controlLaw.name.equals("put-down")){
					handlePutDownCommand(controlLaw);
				} else if(controlLaw.name.equals("put-at-xyz")){
					handlePutAtXYZCommand(controlLaw);
				} else if(controlLaw.name.equals("change-state")){
					handleChangeStateCommand(controlLaw);
				}
	      	}
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }

	private void handlePickUpCommand(control_law_t controlLaw){
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("object-id")){
				Integer objectId = Integer.parseInt(controlLaw.param_values[p].value);
				RosieSimObject obj = getSimObject(objectId);
				if(obj != null){
					robot.pickUpObject(obj);
				} else {
					System.err.println("MobileSimulator::handlePickUpCommand: object-id '" + objectId.toString() + "' not recognized");
				}
			}
		}
	}

	private void handlePutDownCommand(control_law_t controlLaw){
		robot.putDownObject();
	}

	private void handlePutAtXYZCommand(control_law_t controlLaw){
		double[] xyz = new double[]{ 0, 0, 0 };
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("x")){
				xyz[0] = Double.parseDouble(controlLaw.param_values[p].value);
			} else if(controlLaw.param_names[p].equals("y")){
				xyz[1] = Double.parseDouble(controlLaw.param_values[p].value);
			} else if(controlLaw.param_names[p].equals("z")){
				xyz[2] = Double.parseDouble(controlLaw.param_values[p].value);
			} 
		}
		robot.putObjectAtXYZ(xyz);
	}

	private void handleChangeStateCommand(control_law_t controlLaw){
		RosieSimObject obj = null;
		String prop = null;
		String val = null;
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("object-id")){
				Integer objectId = Integer.parseInt(controlLaw.param_values[p].value);
				obj = getSimObject(objectId);
				if(obj == null){
					System.err.println("MobileSimulator::handleChangeStateCommand: object-id '" + objectId.toString() + "' not recognized");
					return;
				}
			} else if(controlLaw.param_names[p].equals("property")){
				prop = controlLaw.param_values[p].value;
			} else if(controlLaw.param_names[p].equals("value")){
				val = controlLaw.param_values[p].value;
			} 
		}
		// TODO: obj.setState(prop, val);
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

		}
    }

}
