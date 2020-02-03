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

import soargroup.mobilesim.util.ResultTypes.*;
import soargroup.mobilesim.sim.*;
import soargroup.rosie.RosieConstants;

import soargroup.mobilesim.sim.actions.*;

// LCM Types
import lcm.lcm.*;
import soargroup.mobilesim.lcmtypes.control_law_t;

public class MobileSimulator implements LCMSubscriber
{
	// Sim stuff
    SimWorld world;
    Simulator sim;

    private Timer simulateDynamicsTimer;
    private static final int DYNAMICS_RATE = 1; // FPS to simulate dynamics at

	SimRobot robot = null;

	private int lastHandledCommand = -1;

    public MobileSimulator(GetOpt opts,
                            VisWorld vw,
                            VisLayer vl,
                            VisCanvas vc,
                            VisConsole console)
    {
        loadWorld(opts);
        sim = new Simulator(vw, vl, console, world);

        ArrayList<SimObject> simObjects;
		ArrayList<RosieSimObject> rosieObjs = new ArrayList<RosieSimObject>();
		synchronized(world.objects){
			simObjects = (ArrayList<SimObject>)world.objects.clone();
		}
		for(SimObject obj : simObjects){
			if(obj instanceof SimRobot){
				robot = (SimRobot)obj;
				robot.setFullyObservable(opts.getBoolean("fully"));
				robot.setupActionRules();
			}
			if(obj instanceof RosieSimObject){
				RosieSimObject rosieObj = (RosieSimObject)obj;
				rosieObjs.add(rosieObj);
				rosieObj.setup(simObjects);
			}
		}
		if(robot == null){
			System.err.println("WARNING: No SimRobot defined in the world file");
		}

	    simulateDynamicsTimer = new Timer();
	    simulateDynamicsTimer.schedule(new SimulateDynamicsTask(rosieObjs, simObjects), 1000, 1000/DYNAMICS_RATE);

		LCM.getSingleton().subscribe("SOAR_COMMAND.*", this);
	}

    public SimWorld getWorld()
    {
    	return world;
    }

	@Override
	public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins) {
        try {
        	if (channel.startsWith("SOAR_COMMAND") && !channel.startsWith("SOAR_COMMAND_STATUS")){
		  		control_law_t controlLaw = new control_law_t(ins);
				if(controlLaw.id == lastHandledCommand){
					return;
				}
				lastHandledCommand = controlLaw.id;

				Action action = null;
				if(controlLaw.name.equals("pick-up")){
					action = parsePickUp(controlLaw);
				} else if(controlLaw.name.equals("put-down")){
					action = parsePutDown(controlLaw);
				} else if(controlLaw.name.equals("put-at-xyz")){
					action = parsePutAtXYZ(controlLaw);
				} else if(controlLaw.name.equals("put-on-object")){
					action = parsePutOnObject(controlLaw);
				} else if(controlLaw.name.equals("change-state")){
					action = parseChangeState(controlLaw);
				} else {
					return;
				}
				if(action == null){
					System.err.println("ERROR: Could not parse action of type " + controlLaw.name);
					return;
				}

				Result result = ActionHandler.handle(action);
				if(result instanceof Err){
					System.err.println(((Err)result).reason);
				}
				System.out.println("Performed: " + action);
	      	}
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
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

	private PickUp parsePickUp(control_law_t controlLaw){
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("object-id")){
				Integer objectId = Integer.parseInt(controlLaw.param_values[p].value);
				RosieSimObject obj = getSimObject(objectId);
				if(obj != null){
					return new PickUp(obj);
				} else {
					System.err.println("MobileSimulator::parsePickUp: object-id '" + objectId.toString() + "' not recognized");
					return null;
				}
			}
		}
		return null;
	}

	private PutDown.Floor parsePutDown(control_law_t controlLaw){
		RosieSimObject obj = robot.getGrabbedObject();
		if(obj == null){
			System.err.println("MobileSimulator::parsePutDown: SimRobot's grabbedObject is null");
			return null;
		}
		return new PutDown.Floor(obj);
	}

	private PutDown.XYZ parsePutAtXYZ(control_law_t controlLaw){
		RosieSimObject obj = robot.getGrabbedObject();
		if(obj == null){
			System.err.println("MobileSimulator::parsePutDown: SimRobot's grabbedObject is null");
			return null;
		}
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
		return new PutDown.XYZ(obj, xyz);
	}

	private PutDown.Target parsePutOnObject(control_law_t controlLaw){
		RosieSimObject grabbedObj = robot.getGrabbedObject();
		if(grabbedObj == null){
			System.err.println("MobileSimulator::parsePutDown: SimRobot's grabbedObject is null");
			return null;
		}
		Integer targetId = null;
		String relation = RosieConstants.REL_ON;
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("object-id")){
				targetId = Integer.parseInt(controlLaw.param_values[p].value);
			} else if(controlLaw.param_names[p].equals("relation")){
				relation = controlLaw.param_values[p].value;
			}
		}
		if(targetId == null){
			System.err.println("MobileSimulator::parsePutOnObject: object-id not given");
			return null;
		}
		RosieSimObject targetObj = getSimObject(targetId);
		if(targetObj != null){
			return new PutDown.Target(grabbedObj, relation, targetObj);
		} else {
			System.err.println("MobileSimulator::parsePutOnObject: object-id '" + targetId.toString() + "' not recognized");
			return null;
		}
	}

	private Action parseChangeState(control_law_t controlLaw){
		RosieSimObject obj = null;
		String prop = null;
		String val = null;
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("object-id")){
				Integer objectId = Integer.parseInt(controlLaw.param_values[p].value);
				obj = getSimObject(objectId);
				if(obj == null){
					System.err.println("MobileSimulator::handleChangeStateCommand: object-id '" + objectId.toString() + "' not recognized");
					return null;
				}
			} else if(controlLaw.param_names[p].equals("property")){
				prop = controlLaw.param_values[p].value;
			} else if(controlLaw.param_names[p].equals("value")){
				val = controlLaw.param_values[p].value;
			} 
		}
		obj.setState(prop, val);
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

    class SimulateDynamicsTask extends TimerTask
    {
		ArrayList<RosieSimObject> rosieObjs;
		ArrayList<SimObject> simObjs;
		public SimulateDynamicsTask(ArrayList<RosieSimObject> rosieObjs, ArrayList<SimObject> simObjects){
			this.rosieObjs = rosieObjs;
			this.simObjs = simObjects;
		}
		@Override
		public void run() {
			for(RosieSimObject obj : rosieObjs){
				obj.performDynamics(simObjs);
			}
		}
    }

}
