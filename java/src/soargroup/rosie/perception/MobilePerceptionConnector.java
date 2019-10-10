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
import soargroup.mobilesim.lcmtypes.ooi_msg_list_t;
import soargroup.mobilesim.lcmtypes.ooi_msg_t;
import soargroup.mobilesim.lcmtypes.svs_info_t;
import soargroup.mobilesim.lcmtypes.svs_object_data_t;
import soargroup.mobilesim.lcmtypes.svs_location_data_t;
import soargroup.mobilesim.lcmtypes.classification_list_t;
import soargroup.mobilesim.lcmtypes.classification_t;
import soargroup.mobilesim.lcmtypes.robot_info_t;
import soargroup.mobilesim.lcmtypes.tag_classification_list_t;
import soargroup.mobilesim.lcmtypes.tag_classification_t;

public class MobilePerceptionConnector extends AgentConnector implements LCMSubscriber{
	private WorldObjectManager objectManager;

	private HashMap<Integer, WorldObject> objects;
	private HashMap<Integer, WorldObject> objsToRemove;
	private boolean newObjectsMessage = false;

	private Integer nextID = 1;

    private LCM lcm;

    private Identifier objectsId = null;

    private Robot robot;

    private final int SVS_UPDATE_INTERVAL_USEC = 100000; // 100 ms update rate
    private boolean sendSvsInfo = false;
    private long lastUpdateSent = 0;

    public MobilePerceptionConnector(SoarAgent agent, Properties props){
    	super(agent);

      sendSvsInfo = props.getProperty("send-svs-info", "false").equals("true");

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
//    	for(ooi_msg_t objMsg : newObjs.observations){
//    		Integer tagID = objMsg.ooi_id;
//    		WorldObject obj = objects.get(tagID);
//    		if(obj == null){
//    			obj = objectManager.getObject(tagID);
//    			if(obj == null){
//    				System.err.println("Bad object tag id from SOAR_OBJECTS: " + tagID.toString());
//    				continue;
//    			}
//    			obj.setHandle(nextID.toString());
//    			nextID++;
//    			objects.put(tagID, obj);
//    		}
//    		obj.update(objMsg.z);
//    	}


    	// Set of objects that didn't appear in the new update
    	// (remove ids as we see them)
    	HashSet<Integer> oldIds = new HashSet<Integer>();
    	oldIds.addAll(objects.keySet());

      double[] robotPose = robot.getXyzrpy();

    	for (ooi_msg_t newObj : newObjs.observations){
        if (newObj.ooi_type != ooi_msg_t.TAG_POSE_QUAT){
           continue;
        }

    		Integer tagID = newObj.ooi_id;
    		WorldObject obj = objects.get(tagID);
    		if(obj != null){
    			// Object is in our normal list, update it
    			oldIds.remove(tagID);
    		} else {
    			obj = objsToRemove.get(tagID);
    			if(obj != null){
    				// Object was going to be removed
    				// Transfer it to the normal list and update
    				objsToRemove.remove(tagID);
    				oldIds.remove(tagID);
    			} else {
    				// It's a new object, add it to the map
    				obj = objectManager.getObject(tagID);
    				if(obj == null){
    					System.err.println("Bad object tag id from SOAR_OBJECTS: " + tagID.toString());
    					continue;
    				}
    				//obj.setHandle(nextID.toString());
    				//nextID++;
    			}
    			objects.put(tagID, obj);
    		}
    		obj.update(robotPose, newObj.data);
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

    	// Update SVS
    	updateSVS();

      if(sendSvsInfo){
        sendObservations();
      }

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
//    	Region curRegion = robot.getRegion();
//    	if(curRegion != null){
//    		for(WorldObject obj : objects.values()){
//    			Collection<Region> regions = robot.getMapInfo().getRegions(obj.getPos());
//    			if(regions.contains(curRegion)){
//    				// in current location
//    				if(obj.isAdded()){
//    					obj.updateWM();
//    				} else {
//    					obj.addToWM(objectsId);
//    				}
//    			} else {
//    				// not in current location
//    				if(obj.isAdded()){
//    					obj.removeFromWM();
//    				}
//    			}
//
//    		}
//
//    	}
    	//for(WorldObject obj : objects.values()){
    	//	if(obj.isAdded()){
    	//		obj.updateWM();
    	//	} else {
    	//		obj.addToWM(objectsId);
    	//	}
    	//}
    	for(WorldObject obj : objsToRemove.values()){
    		obj.removeFromWM();
    	}
    }

    private void sendObservations(){
      if ((TimeUtil.utime() - lastUpdateSent) < SVS_UPDATE_INTERVAL_USEC){
        return;
      }
    	ArrayList<svs_object_data_t> objDatas = new ArrayList<svs_object_data_t>();
    	
    	String[] objs = soarAgent.getAgent().SVSQuery("list-all-objs\n").split(" ");
    	for(int i = 2; i < objs.length; i++){
    		String id = objs[i].trim();
    		if(id.length() == 0){
    			continue;
    		}
    		String objInfo = soarAgent.getAgent().SVSQuery("obj-info " + id);
            svs_object_data_t obj = parseObject(objInfo);
            if(obj.id.equals("world") || obj.id.equals("robot_pos") || 
                    obj.id.equals("robot_view") || obj.id.equals("robot_body")){
                continue;
            }
            Boolean loc = false;
            for(String label : obj.labels){
                if(label.equals("category=location")){
                    loc = true;
                }
            }
            if(!loc){
                objDatas.add(obj);
            }
    	}

        ArrayList<svs_location_data_t> locDatas = new ArrayList<svs_location_data_t>();
        //HashSet<Region> regions = robot.getMapInfo().getAllRegions();
//        for(Region reg : regions){
//            locDatas.add(reg.getLcmData());
//        }
//
    	svs_info_t svsInfo = new svs_info_t();
    	svsInfo.utime = TimeUtil.utime();
    	svsInfo.nobjects = objDatas.size();
        svsInfo.objects = objDatas.toArray(new svs_object_data_t[objDatas.size()]);
    	svsInfo.nlocations = locDatas.size();
        svsInfo.locations = locDatas.toArray(new svs_location_data_t[locDatas.size()]);
    	
    	LCM.getSingleton().publish("SVS_INFO", svsInfo);

      lastUpdateSent = TimeUtil.utime();
    }
    
    public svs_object_data_t parseObject(String objInfo){
      svs_object_data_t objData = new svs_object_data_t();

    	objData.utime = TimeUtil.utime();
    	objData.xyzrpy = new double[6];
    	objData.lwh = new double[3];

      ArrayList<String> tags = new ArrayList<String>();

    	String[] fields = objInfo.trim().split(" ");
    	int i = 0;
    	while(i < fields.length){
    		String field = fields[i++];
    		if(field.equals("o")){
    			// Parse ID: Should be in format bel-#
          objData.id = fields[i++];
    		} else if (field.equals("p") || field.equals("r") || field.equals("s")){
    			// Parse position, rotation, or scaling
          for(int d = 0; d < 3; d++){
            double val = Double.parseDouble(fields[i++]);
            if(field.equals("p")){
              objData.xyzrpy[d] = val;
            } else if(field.equals("r")){
              objData.xyzrpy[3+d] = val;
            } else if(field.equals("s")){
              objData.lwh[d] = val;
            }
    			}
    		} else if(field.equals("t")){
    			Integer numTags = Integer.parseInt(fields[i++]);
          for(int t = 0; t < numTags; t++){
            tags.add(fields[i++] + "=" + fields[i++]);  // "tag_name=tag_value"
          }
    		}
    	}

      objData.nlabels = tags.size();
      objData.labels = tags.toArray(new String[tags.size()]);
    	
    	return objData;
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
