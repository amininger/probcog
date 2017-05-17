package probcog.rosie.actuation;

import java.util.Collection;
import java.util.HashMap;
import java.util.Properties;

import edu.umich.rosie.soar.AgentConnector;
import edu.umich.rosie.soar.SoarAgent;
import edu.umich.rosie.soar.SoarUtil;
import probcog.lcmtypes.control_law_t;
import probcog.rosie.perception.Robot;
import probcog.rosie.perception.SimMobilePerceptionConnector;
import probcog.rosie.perception.WorldObject;
import sml.Identifier;

import javax.swing.JMenuBar;

public class SimMobileActuationConnector extends AgentConnector {
	private Robot robot;
	private HashMap<String, WorldObject> objects = new HashMap<String, WorldObject>();
	
    public SimMobileActuationConnector(SoarAgent agent, Robot robot, Properties props){
    	super(agent);
    	
    	this.robot = robot;

        // Setup Output Link Events
        String[] outputHandlerStrings = { "do-control-law", "stop", "face-point", "pick-up", "put-down"};
        this.setOutputHandlerNames(outputHandlerStrings);
    }
    
    @Override
    public void connect(){
    	for(WorldObject obj : ((SimMobilePerceptionConnector)soarAgent.getPerceptionConnector()).getObjects()){
    		objects.put(obj.getHandle(), obj);
    	}
    	super.connect();
    }
    
    @Override
    public void disconnect(){
    	super.disconnect();
    }
    
    public String getMovingState(){
    	return "stopped";
    }

	public void createMenu(JMenuBar menuBar) { }

    
    /*******************************************************
     * Methods for updating the input link
     ******************************************************/

	protected void onInputPhase(Identifier inputLink) {

	}
	
	protected void onInitSoar() { }
	
	/*************************************************************
	 * 
	 * Handling Commands on the Soar Output-Link
	 ************************************************************/

	protected void onOutputEvent(String attName, Identifier id) {
		if(attName.equals("do-control-law")){
			processDoControlLawCommand(id);
		} else if(attName.equals("stop")){
			processStopCommand(id);
		} else if(attName.equals("face-point")){
			processFacePoint(id);
		} else if(attName.equals("pick-up")){
			
		} else if(attName.equals("pick-down")){
			
		}
	}

    public void processDoControlLawCommand(Identifier id){
    	control_law_t controlLaw = SoarCommandParser.parseControlLaw(id);
    	if(controlLaw == null){
    		id.CreateStringWME("status", "error");
    		id.CreateStringWME("error-type", "syntax-error");
    		return;
    	}
    	
    	if(controlLaw.name.equals("pick-up")){
    		for(int i = 0; i < controlLaw.num_params; i++){
    			if(controlLaw.param_names[i].equals("object-id")){
    				String objId = controlLaw.param_values[i].value;
    				
    				break;
    			}
    		}
    	} else if (controlLaw.name.equals("put-down")){
    		
    	} else if (controlLaw.name.equals("drive-xy")){
    		double x = 0.0;
    		double y = 0.0;
    		for(int i = 0; i < controlLaw.num_params; i++){
    			if(controlLaw.param_names[i].equals("x")){
    				x = new Double(controlLaw.param_values[i].value);
    			}
    			if(controlLaw.param_names[i].equals("y")){
    				y = new Double(controlLaw.param_values[i].value);
    			}
    		}
    		robot.setPos(new double[]{x, y});
    		if(!robot.getHeldObject().equals("None") && objects.get(robot.getHeldObject()) != null){
    			WorldObject obj = objects.get(objects.get(robot.getHeldObject()));
    			obj.setPos(new double[]{x, y, 0});
    		}
    	}

    	SoarUtil.updateStringWME(id, "status", "success");
    }

    public void processStopCommand(Identifier id){
    	SoarUtil.updateStringWME(id, "status", "success");
    }
    
    public void processFacePoint(Identifier id){
    	SoarUtil.updateStringWME(id, "status", "success");
    }
}
