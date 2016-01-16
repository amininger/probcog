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
	private boolean newClassifications = false;
	private tag_classification_list_t curTagClassifications = null;
	private Identifier waypointId = null;
	private int curWaypoint = -1;
	
	private HashMap<String, InputLinkObject> objects;
	private HashMap<String, InputLinkObject> objsToRemove;
	private boolean newObjectsMessage = false;

    private LCM lcm;
    
    private Identifier selfId = null;
    private Identifier objectsId = null;
    
    private boolean newPose = false;
    private Pose pose;

    public MobilePerceptionConnector(SoarAgent agent, Properties props){
    	super(agent);
    	
    	objects = new HashMap<String, InputLinkObject>();
    	objsToRemove = new HashMap<String, InputLinkObject>();
    	
    	pose = new Pose();

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
				curTagClassifications = new tag_classification_list_t(ins);
				newClassifications = true;
			} else if (channel.startsWith("POSE")){
				pose_t p = new pose_t(ins);
				pose.updatePosition(p.pos);
				newPose = true;
			} else if(channel.startsWith("DETECTED_OBJECTS")){
				soar_objects_t newObjs = new soar_objects_t(ins);
				processNewObjectMessage(newObjs);
				newObjectsMessage = true;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    

    private void processNewObjectMessage(soar_objects_t newObjs){
    	// Set of objects that didn't appear in the new update
    	// (remove ids as we see them)
    	HashSet<String> oldIds = new HashSet<String>();
    	oldIds.addAll(objects.keySet());

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
    	}
    	
    	for(String oldID : oldIds){
    		InputLinkObject oldObj = objects.get(oldID);
    		objects.remove(oldID);
    		objsToRemove.put(oldID, oldObj);
    	}
    }
    
	/***************************
	 * 
	 * INPUT PHASE HANDLING
	 * 
	 **************************/
    
    protected synchronized void onInputPhase(Identifier inputLink){
    	// Update the information about the robot
    	if(selfId == null){
    		selfId = inputLink.CreateIdWME("self");
    	}
    	updateSelf();

    	// Update information about waypoints
    	updateClassifications();

    	// Update information about objects
    	if(objectsId == null){
    		objectsId = inputLink.CreateIdWME("objects");
    	}
    	updateObjects();
    }
    
    private void updateSelf(){
    	if(!pose.isAdded()){
    		pose.addToWM(selfId);
    	} else if(newPose){
    		pose.updateWM();
    		newPose = false;
    	}

    	String movingState = ((MobileActuationConnector)soarAgent.getActuationConnector()).getMovingState();
		SoarUtil.updateStringWME(selfId, "moving-state", movingState);
    }
    
    private void updateClassifications(){
    	if(!newClassifications){
    		return;
    	}
    	newClassifications = false;
    	
    	int closestWaypoint = -1;
    	double closestDistance = Double.MAX_VALUE;
    	for (tag_classification_t c : curTagClassifications.classifications){
    		if (c.range < closestDistance){
    			closestWaypoint = c.tag_id;
    			closestDistance = c.range;
    		}
    	}
   		if (closestWaypoint != curWaypoint && waypointId != null){
   			waypointId.DestroyWME();
   			waypointId = null;
   		}
   		curWaypoint = closestWaypoint;
   		if (curWaypoint == -1){
   			return;
   		}
   		if (waypointId == null){
    		waypointId = selfId.CreateIdWME("current-waypoint");
   		}
   		for (tag_classification_t c : curTagClassifications.classifications){
   			if (c.tag_id == closestWaypoint){
   				if (c.name.startsWith("wp")){
   					SoarUtil.updateStringWME(waypointId, "waypoint-handle", c.name);
   				} else {
   					SoarUtil.updateStringWME(waypointId, "classification", c.name);
   				}
   			}
   		}
    }
    
    private void updateObjects(){
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
    	objsToRemove.clear();
    }


	@Override
	protected void onInitSoar() {
		pose.removeFromWM();
		for(InputLinkObject obj : objects.values()){
			obj.removeFromWM();
		}
		if(objectsId != null){
			objectsId.DestroyWME();
			objectsId = null;
		}
		if(selfId != null){
			selfId.DestroyWME();
			selfId = null;
			waypointId = null;
		}
	}
	
    // Currently no commands relevant to perception
	protected void onOutputEvent(String attName, Identifier id) { }
}
