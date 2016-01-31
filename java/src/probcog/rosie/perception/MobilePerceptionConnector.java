package probcog.rosie.perception;

import java.io.IOException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Properties;

import javax.swing.JMenuBar;

import edu.umich.rosie.soar.AgentConnector;
import edu.umich.rosie.soar.SoarAgent;
import edu.umich.rosie.soar.SoarUtil;
import edu.umich.rosie.soarobjects.Pose;
import april.jmat.LinAlg;
import april.lcmtypes.pose_t;
import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import probcog.lcmtypes.classification_list_t;
import probcog.lcmtypes.classification_t;
import probcog.lcmtypes.object_data_t;
import probcog.lcmtypes.soar_objects_t;
import probcog.lcmtypes.tag_classification_list_t;
import probcog.lcmtypes.tag_classification_t;
import probcog.rosie.actuation.MobileActuationConnector;
import sml.Identifier;

public class MobilePerceptionConnector extends AgentConnector implements LCMSubscriber{

	private HashMap<String, InputLinkObject> objects;
	private HashMap<String, InputLinkObject> objsToRemove;
	private boolean newObjectsMessage = false;

    private LCM lcm;
    
    private Identifier objectsId = null;
    
    private Robot robot;
    
    public MobilePerceptionConnector(SoarAgent agent, Properties props){
    	super(agent);
    	
    	objects = new HashMap<String, InputLinkObject>();
    	objsToRemove = new HashMap<String, InputLinkObject>();
    	
    	robot = new Robot();
    	
    	// Setup LCM events
        lcm = LCM.getSingleton();
    }
    
    @Override
    public void connect(){
    	super.connect();
        lcm.subscribe("CLASSIFICATIONS.*", this);
        lcm.subscribe("POSE", this);
        lcm.subscribe("DETECTED_OBJECTS", this);
    }
    
    @Override
    public void disconnect(){
    	super.disconnect();
        lcm.unsubscribe("CLASSIFICATIONS.*", this);
        lcm.unsubscribe("POSE", this);
        lcm.unsubscribe("DETECTED_OBJECTS", this);
    }

	@Override
	public void createMenu(JMenuBar menuBar) {}

	/***************************
	 * 
	 * LCM HANDLING
	 * 
	 **************************/
	
	
    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
		try {
			if(channel.startsWith("CLASSIFICATIONS")){
				tag_classification_list_t tcl = new tag_classification_list_t(ins);
				handleClassificationsMessage(tcl);
			} else if (channel.startsWith("POSE")){
				pose_t p = new pose_t(ins);
				handlePoseMessage(p);
			} else if(channel.startsWith("DETECTED_OBJECTS")){
				soar_objects_t newObjs = new soar_objects_t(ins);
				processNewObjectMessage(newObjs);
				newObjectsMessage = true;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    
    private void handleClassificationsMessage(tag_classification_list_t tcl){
    	robot.updateClassifications(tcl);
    }
    
    private void handlePoseMessage(pose_t pose){
    	double[] xyzrpy = LinAlg.quatPosToXyzrpy(pose.orientation, pose.pos);
    	robot.updatePose(xyzrpy);
    }

    private void processNewObjectMessage(soar_objects_t newObjs){
    	// Set of objects that didn't appear in the new update
    	// (remove ids as we see them)
    	HashSet<String> oldIds = new HashSet<String>();
    	oldIds.addAll(objects.keySet());
    	
    	String heldObject = "none";

    	for (object_data_t newObj : newObjs.objects){
    		String objID = newObj.id;
    		InputLinkObject obj = objects.get(objID);
    		if(obj != null){
    			// Object is in our normal list, update it
    			obj.update(newObj);
    			oldIds.remove(objID);
    		} else {
    			obj = objsToRemove.get(objID);
    			if(obj != null){
    				// Object was going to be removed
    				// Transfer it to the normal list and update
    				objsToRemove.remove(objID);
    				objects.put(objID, obj);
    				obj.update(newObj);
    				oldIds.remove(objID);
    			} else {
    				// It's a new object, add it to the map
    				obj = new InputLinkObject(newObj);
    				objects.put(objID, obj);
    			}
    		}
    		
    		for(classification_t cls : newObj.classifications){
    			if(cls.category.equals("arm-status") && cls.name.equals("grabbed")){
    				heldObject = objID;
    			}
    		}
    	}
    	
    	for(String oldID : oldIds){
    		InputLinkObject oldObj = objects.get(oldID);
    		objects.remove(oldID);
    		objsToRemove.put(oldID, oldObj);
    	}
    	
    	robot.setHeldObject(heldObject);
    }
    
	/***************************
	 * 
	 * INPUT PHASE HANDLING
	 * 
	 **************************/
    
    protected synchronized void onInputPhase(Identifier inputLink){
    	// Update the information about the robot
    	updateRobot();

    	// Update information about objects
    	updateObjects();
    	
    	// Update SVS
    	updateSVS();
    	
    	objsToRemove.clear();
    }
    
    private void updateRobot(){
    	robot.updateMovingState(((MobileActuationConnector)soarAgent.getActuationConnector()).getMovingState());
    	if(!robot.isAdded()){
    		robot.addToWM(soarAgent.getAgent().GetInputLink());
    	} else {
    		robot.updateWM();
    	}
    }
    
    private void updateObjects(){
    	if(objectsId == null){
    		objectsId = soarAgent.getAgent().GetInputLink().CreateIdWME("objects");
    	}
    	for(InputLinkObject obj : objects.values()){
    		if(obj.isAdded()){
    			obj.updateWM();
    		} else {
    			obj.addToWM(objectsId);
    		}
    	}
    	for(InputLinkObject obj : objsToRemove.values()){
    		obj.removeFromWM();
    	}
    }
    
    private void updateSVS(){
    	StringBuilder svsCommands = new StringBuilder();
    	svsCommands.append(robot.getSVSCommands());
    	for(InputLinkObject obj : objects.values()){
    		svsCommands.append(obj.getSVSCommands());
    	}
    	for(InputLinkObject obj : objsToRemove.values()){
    		svsCommands.append(obj.getSVSCommands());
    	}
    	if(svsCommands.length() > 0){
    		soarAgent.getAgent().SendSVSInput(svsCommands.toString());
    	}
    }

	@Override
	protected void onInitSoar() {
		for(InputLinkObject obj : objects.values()){
			obj.removeFromWM();
		}
		for(InputLinkObject obj : objsToRemove.values()){
			obj.removeFromWM();
		}
		if(objectsId != null){
			objectsId.DestroyWME();
			objectsId = null;
		}
		robot.removeFromWM();
		updateSVS();
	}
	
    // Currently no commands relevant to perception
	protected void onOutputEvent(String attName, Identifier id) { }
}
