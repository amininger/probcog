package soargroup.rosie.perception;

import java.io.IOException;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.Properties;

import java.nio.ByteBuffer;

import javax.swing.JMenuBar;

import edu.umich.rosie.soar.AgentConnector;
import edu.umich.rosie.soar.SoarAgent;
import edu.umich.rosie.soar.SoarUtil;
import edu.umich.rosie.soarobjects.Pose;
import april.jmat.LinAlg;
import april.lcmtypes.pose_t;
import april.util.*;

import sml.Identifier;
import soargroup.rosie.actuation.MobileActuationConnector;

// LCM Types
import lcm.lcm.*;
import soargroup.mobilesim.lcmtypes.object_data_t;
import soargroup.mobilesim.lcmtypes.object_data_list_t;
import soargroup.mobilesim.lcmtypes.robot_info_t;

public class MobilePerceptionConnector extends AgentConnector implements LCMSubscriber{
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
				object_data_list_t newObjs = new object_data_list_t(ins);
				processNewObjectMessage(newObjs);
				newObjectsMessage = true;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }

    private void processNewObjectMessage(object_data_list_t newObjs){
    	// Set of objects that didn't appear in the new update
    	// (remove ids as we see them)
    	HashSet<Integer> oldIds = new HashSet<Integer>();
    	oldIds.addAll(objects.keySet());

		for(object_data_t newObj : newObjs.objects){
			Integer objID = newObj.id;
			if(oldIds.contains(objID)){
				oldIds.remove(objID);
			}

			WorldObject obj = objects.get(objID);
			if(obj == null){
				if(objsToRemove.containsKey(objID)){
    				// Object was going to be removed
    				// Transfer it to the normal list and update
					obj = objsToRemove.get(objID);
					objsToRemove.remove(objID);
				} else {
    				// It's a new object, add it to the map
					obj = new WorldObject(newObj);
				}
			}

    		obj.update(newObj);
    	}

    	for(Integer oldID : oldIds){
    		WorldObject oldObj = objects.get(oldID);
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
    	updateRobot();

    	// Update information about objects
    	updateObjects();
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
    	StringBuilder svsCommands = new StringBuilder();
		for(WorldObject obj : objects.values()){
			if(obj.isAdded()){
				obj.updateWM(svsCommands);
			} else {
				obj.addToWM(objectsId, svsCommands);
			}
		}
    	for(WorldObject obj : objsToRemove.values()){
    		obj.removeFromWM(svsCommands);
    	}
    	if(svsCommands.length() > 0){
    		soarAgent.getAgent().SendSVSInput(svsCommands.toString());
    	}
    	objsToRemove.clear();
    }

	@Override
	protected void onInitSoar() {
    	StringBuilder svsCommands = new StringBuilder();
		for(WorldObject obj : objects.values()){
			obj.removeFromWM(svsCommands);
		}
		for(WorldObject obj : objsToRemove.values()){
			obj.removeFromWM(svsCommands);
		}
    	if(svsCommands.length() > 0){
    		soarAgent.getAgent().SendSVSInput(svsCommands.toString());
    	}
		if(objectsId != null){
			objectsId.DestroyWME();
			objectsId = null;
		}
		robot.removeFromWM();
	}

    // Currently no commands relevant to perception
	protected void onOutputEvent(String attName, Identifier id) { }
}
