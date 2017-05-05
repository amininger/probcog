package probcog.rosie.perception;

import java.util.HashMap;
import java.util.HashSet;
import java.util.ArrayList;
import java.util.Collection;
import java.util.Properties;

import javax.swing.JMenuBar;

import edu.umich.rosie.soar.AgentConnector;
import edu.umich.rosie.soar.SoarAgent;
import april.util.*;
import lcm.lcm.LCM;
import magic2.lcmtypes.svs_info_t;
import magic2.lcmtypes.svs_object_data_t;
import magic2.lcmtypes.svs_location_data_t;
import probcog.rosie.actuation.SimMobileActuationConnector;
import sml.Identifier;

public class SimMobilePerceptionConnector extends AgentConnector {
	private WorldObjectManager objectManager;
	
	private HashMap<Integer, WorldObject> objects;
	private HashSet<WorldObject> visibleObjects;

    private Identifier objectsId = null;

    private Robot robot;

    private final int SVS_UPDATE_INTERVAL_USEC = 100000; // 100 ms update rate
    private boolean sendSvsInfo = false;
    private long lastUpdateSent = 0;
    
    public SimMobilePerceptionConnector(SoarAgent agent, Properties props){
    	super(agent);
    	
    	sendSvsInfo = props.getProperty("send-svs-info", "false").equals("true");

    	objects = new HashMap<Integer, WorldObject>();
    	visibleObjects = new HashSet<WorldObject>();

    	objectManager = new WorldObjectManager(props);
    	HashSet<WorldObject> allObjs = objectManager.getObjects();
    	for(WorldObject obj : allObjs){
    		objects.put(obj.getTagID(), obj);
    	}

    	robot = new Robot(props);
    }
    
    public Robot getRobot(){
    	return robot;
    }
    
    public Collection<WorldObject> getObjects(){
    	return new HashSet<WorldObject>(objects.values());
    }

    @Override
    public void connect(){
    	super.connect();
    }

    @Override
    public void disconnect(){
    	super.disconnect();
    }

	@Override
	public void createMenu(JMenuBar menuBar) {}

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

    }

    private void updateRobot(){
    	robot.updateMovingState(((SimMobileActuationConnector)soarAgent.getActuationConnector()).getMovingState());
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
    	if(curRegion == null){
    		return;
    	}
    	
		for(WorldObject obj : objects.values()){
			System.out.println(obj.getHandle());
			System.out.println(java.util.Arrays.toString(obj.getPos()));
			for(Region reg : robot.getMapInfo().getRegions(obj.getPos())){
				System.out.println("  " + reg.label);
			}
			boolean inRegion = robot.getMapInfo().getRegions(obj.getPos()).contains(curRegion);
			boolean visible = visibleObjects.contains(obj);
			
			if(inRegion && !visible){
				obj.addToWM(objectsId);
				visibleObjects.add(obj);
			} else if(inRegion && visible){
				obj.updateWM();
			} else if(!inRegion && visible){
				obj.removeFromWM();
				visibleObjects.remove(obj);
			}
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
        HashSet<Region> regions = robot.getMapInfo().getAllRegions();
        for(Region reg : regions){
            locDatas.add(reg.getLcmData());
        }

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
    	if(svsCommands.length() > 0){
    		soarAgent.getAgent().SendSVSInput(svsCommands.toString());
    	}
    }

	@Override
	protected void onInitSoar() {
		for(WorldObject obj : objects.values()){
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
