package probcog.rosie.perception;

import java.io.IOException;
import java.util.Collection;
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
import magic2.lcmtypes.ooi_msg_list_t;
import magic2.lcmtypes.ooi_msg_t;
import probcog.lcmtypes.classification_list_t;
import probcog.lcmtypes.classification_t;
import probcog.lcmtypes.object_data_t;
import probcog.lcmtypes.robot_info_t;
import probcog.lcmtypes.tag_classification_list_t;
import probcog.lcmtypes.tag_classification_t;
import probcog.rosie.actuation.MobileActuationConnector;
import sml.Identifier;

public class MobilePerceptionConnector extends AgentConnector implements LCMSubscriber{
	private WorldObjectManager objectManager;

	private HashMap<Integer, WorldObject> objects;
	private HashMap<Integer, WorldObject> objsToRemove;
	private boolean newObjectsMessage = false;
	
	private Integer nextID = 1;

    private LCM lcm;
    
    private Identifier objectsId = null;
    
    private Robot robot;
    
    public MobilePerceptionConnector(SoarAgent agent, Properties props){
    	super(agent);
    	
    	objects = new HashMap<Integer, WorldObject>();
    	objsToRemove = new HashMap<Integer, WorldObject>();
    	
    	objectManager = new WorldObjectManager(props);
    	
    	robot = new Robot(props);
    	
    	// Setup LCM events
        lcm = LCM.getSingleton();
    }
    
    @Override
    public void connect(){
    	super.connect();
        lcm.subscribe("ROBOT_INFO", this);
        lcm.subscribe("DETECTED_OBJECTS", this);
    }
    
    @Override
    public void disconnect(){
    	super.disconnect();
        lcm.unsubscribe("ROBOT_INFO", this);
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
			if (channel.startsWith("ROBOT_INFO")){
				robot_info_t robotInfo = new robot_info_t(ins);
				robot.update(robotInfo);
				if(objects.containsKey(robotInfo.held_object)){
					robot.setHeldObject(objects.get(robotInfo.held_object).getHandle());
				} else {
					robot.setHeldObject(null);
				}
			} else if(channel.startsWith("DETECTED_OBJECTS")){
				ooi_msg_list_t newObjs = new ooi_msg_list_t(ins);
				processNewObjectMessage(newObjs);
				newObjectsMessage = true;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }

    private void processNewObjectMessage(ooi_msg_list_t newObjs){
    	for(ooi_msg_t objMsg : newObjs.observations){
    		Integer tagID = objMsg.ooi_id;
    		WorldObject obj = objects.get(tagID);
    		if(obj == null){
    			obj = objectManager.getObject(tagID);
    			if(obj == null){
    				System.err.println("Bad object tag id from SOAR_OBJECTS: " + tagID.toString());
    				continue;
    			}
    			obj.setHandle(nextID.toString());
    			nextID++;
    			objects.put(tagID, obj);
    		}
    		obj.update(objMsg.z);
    	}
    	
    	
//    	// Set of objects that didn't appear in the new update
//    	// (remove ids as we see them)
//    	HashSet<Integer> oldIds = new HashSet<Integer>();
//    	oldIds.addAll(objects.keySet());
//    	
//    	for (ooi_msg_t newObj : newObjs.observations){
//    		Integer tagID = newObj.ooi_id;
//    		WorldObject obj = objects.get(tagID);
//    		if(obj != null){
//    			// Object is in our normal list, update it
//    			oldIds.remove(tagID);
//    		} else {
//    			obj = objsToRemove.get(tagID);
//    			if(obj != null){
//    				// Object was going to be removed
//    				// Transfer it to the normal list and update
//    				objsToRemove.remove(tagID);
//    				oldIds.remove(tagID);
//    			} else {
//    				// It's a new object, add it to the map
//    				obj = objectManager.getObject(tagID);
//    				if(obj == null){
//    					System.err.println("Bad object tag id from SOAR_OBJECTS: " + tagID.toString());
//    					continue;
//    				}
//    				obj.setHandle(nextID.toString());
//    				nextID++;
//    			}
//    			objects.put(tagID, obj);
//    		}
//    		obj.update(newObj.z);
//    	}
//    	
//    	for(Integer oldID : oldIds){
//    		WorldObject oldObj = objects.get(oldID);
//    		objects.remove(oldID);
//    		objsToRemove.put(oldID, oldObj);
//    	}
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
    	Region curRegion = robot.getRegion();
    	if(curRegion != null){
    		for(WorldObject obj : objects.values()){
    			Collection<Region> regions = robot.getMapInfo().getRegions(obj.getPos());
    			if(regions.contains(curRegion)){
    				// in current location
    				if(obj.isAdded()){
    					obj.updateWM();
    				} else {
    					obj.addToWM(objectsId);
    				}
    			} else {
    				// not in current location
    				if(obj.isAdded()){
    					obj.removeFromWM();
    				}
    			}
    
    		}
    		
    	}
//    	for(WorldObject obj : objects.values()){
//    		if(obj.isAdded()){
//    			obj.updateWM();
//    		} else {
//    			obj.addToWM(objectsId);
//    		}
//    	}
//    	for(WorldObject obj : objsToRemove.values()){
//    		obj.removeFromWM();
//    	}
    }
    
    private void updateSVS(){
    	StringBuilder svsCommands = new StringBuilder();
    	svsCommands.append(robot.getSVSCommands());
    	for(WorldObject obj : objects.values()){
    		svsCommands.append(obj.getSVSCommands());
    	}
    	for(WorldObject obj : objsToRemove.values()){
    		svsCommands.append(obj.getSVSCommands());
    	}
    	if(svsCommands.length() > 0){
    		soarAgent.getAgent().SendSVSInput(svsCommands.toString());
    	}
    }

	@Override
	protected void onInitSoar() {
		for(WorldObject obj : objects.values()){
			obj.removeFromWM();
		}
		for(WorldObject obj : objsToRemove.values()){
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
