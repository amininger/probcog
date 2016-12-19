package probcog.rosie.perception;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Set;

//x import lcm.lcm.LCM;
import sml.Agent.RunEventInterface;
import sml.Agent;
import sml.Identifier;
import sml.smlRunEventId;
//x import probcog.lcmtypes.*;
import april.util.TimeUtil;
import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.SVSCommands;
import edu.umich.rosie.soar.SoarAgent;

import javax.json.*;

public class WorldModel implements ISoarObject
{
    private SoarAgent soarAgent;

    // Mapping from object ids to objects
	private Identifier objectsId;
    private Map<Integer, WorldObject> objects;
    private HashSet<WorldObject> objsToRemove;

    private double[] eyePos;

    private HashMap<Integer, Integer> objectLinks;

    private boolean needsUpdate = false;

    public WorldModel(SoarAgent soarAgent){
    	this.soarAgent = soarAgent;

        objects = new HashMap<Integer, WorldObject>();
        objsToRemove = new HashSet<WorldObject>();
        objectLinks = new HashMap<Integer, Integer>();

        eyePos = new double[6];
    }

    public Agent getAgent(){
    	return soarAgent.getAgent();
    }

    public void reset(){
    	for(WorldObject object : objects.values()){
    		String command = SVSCommands.delete(object.getHandleStr());
    		soarAgent.getAgent().SendSVSInput(command);
    	}

    	objects.clear();
    }

    public synchronized Integer getPerceptionId(Integer handle){
    	if(objects.containsKey(handle)){
    		return objects.get(handle).getPerceptionId();
    	}
    	return null;
    }

    public synchronized Integer getSoarHandle(Integer handle){
    	if(objectLinks.containsKey(handle)){
    		return objectLinks.get(handle);
    	}
    	return handle;
    }


    public synchronized void linkObjects(Set<String> sourceHandles, String destHandle){
    	Integer dHandle = Integer.parseInt(destHandle);

    	//ArrayList<object_data_t> objData = new ArrayList<object_data_t>();

    	for(String sourceHandle : sourceHandles){
    		Integer sHandle = Integer.parseInt(sourceHandle);
    		objectLinks.put(sHandle, dHandle);
    		if(objects.containsKey(sHandle)){
    			WorldObject wobj = objects.get(sHandle);
    			//objData.addAll(wobj.getLastDatas());
    			objsToRemove.add(wobj);
    			objects.remove(sHandle);
    		}
    	}


        // if(objData.size() > 0){
        // 	if(objects.containsKey(dHandle)){
        // 		WorldObject dObj = objects.get(dHandle);
        // 		objData.addAll(dObj.getLastDatas());
    	// 		dObj.update(objData);
    	// 	} else {
    	// 		WorldObject object = new WorldObject(this, dHandle, objData);
    	// 		objects.put(dHandle, object);
    	// 	}
        // }
        needsUpdate = true;
    }

    public synchronized void newObservation(JsonObject observation) {
        JsonArray i = observation.getJsonArray("eye");
        for (int n = 0; n < 3; n++) {
            eyePos[n] = i.getJsonNumber(n).doubleValue();
        }

    	Set<Integer> staleObjects = new HashSet<Integer>();
    	for(WorldObject object : objects.values()){
    		staleObjects.add(object.getHandle());
    	}

    	// Combine multiple observations that correspond to the same object into a list per id
    	HashMap<Integer, ArrayList<ObjectData>> newData =
            new HashMap<Integer, ArrayList<ObjectData>>();

        ArrayList<ObjectData> tmp = new ArrayList<ObjectData>();

        try {
            JsonArray obses = observation.getJsonArray("observations");
            for (int j = 0; j < obses.size(); j++) {
                tmp.add(new ObjectData(obses.getJsonObject(j)));
            }
        } catch (Exception e) {
            System.out.println("ERROR: Reading JSON observations");
        }

    	for(ObjectData objData : tmp){
    		Integer handle = objData.getID();
    		if(objectLinks.containsKey(handle)){
    			handle = objectLinks.get(handle);
    		}
    		if(!newData.containsKey(handle)){
    			newData.put(handle, new ArrayList<ObjectData>());
    		}
    		newData.get(handle).add(objData);
    	}

    	// // For each object, either update existing or create if new
    	for(Map.Entry<Integer, ArrayList<ObjectData>> e : newData.entrySet()){
    		Integer handle = e.getKey();
    		WorldObject object = objects.get(handle);
    		if(object == null){
    			object = new WorldObject(this, handle, e.getValue());
    			objects.put(handle, object);
    		} else {
    			staleObjects.remove(handle);
    			object.update(e.getValue());
    		}
    	}

    	// // Remove all stale objects from WM
        for(Integer handle : staleObjects){
        	WorldObject object = objects.get(handle);
        	objsToRemove.add(object);
        	objects.remove(handle);
        }

        needsUpdate = true;
        System.out.println("Processed obs");
    }

    // private void sendObservation(){
    // 	ArrayList<object_data_t> objDatas = new ArrayList<object_data_t>();

    // 	String[] beliefObjects = soarAgent.getAgent().SVSQuery("objs-with-flag object-source belief\n").split(" ");
    // 	for(int i = 2; i < beliefObjects.length; i++){
    // 		String beliefId = beliefObjects[i].trim();
    // 		if(beliefId.length() == 0){
    // 			continue;
    // 		}
    // 		String obj = soarAgent.getAgent().SVSQuery("obj-info " + beliefId);
    // 		objDatas.add(parseObject(obj));
    // 	}

    	// soar_objects_t outgoingObs = new soar_objects_t();
    	// outgoingObs.utime = TimeUtil.utime();
    	// outgoingObs.objects = objDatas.toArray(new object_data_t[objDatas.size()]);
    	// outgoingObs.num_objects = outgoingObs.objects.length;

    	//x LCM.getSingleton().publish("SOAR_OBJECTS", outgoingObs);
    // }

    // public object_data_t parseObject(String objInfo){
    // 	object_data_t objData = new object_data_t();
    // 	objData.utime = TimeUtil.utime();
    // 	objData.bbox_xyzrpy = new double[6];
    // 	objData.bbox_dim = new double[3];
    // 	objData.pos = new double[6];
    // 	objData.num_cat = 0;
    // 	objData.cat_dat = new categorized_data_t[0];
    // 	objData.num_states = 0;
    // 	objData.state_values = new String[0];

    // 	String[] fields = objInfo.split(" ");
    // 	int i = 0;
    // 	while(i < fields.length){
    // 		String field = fields[i++];
    // 		if(field.equals("o")){
    // 			// Parse ID: Should be in format bel-#
    // 			String beliefId = fields[i++];
    // 			Integer index = beliefId.indexOf("bel-");
    // 			if(index == 0){
    // 				objData.id = Integer.parseInt(beliefId.substring(4));
    // 			} else {
    // 				objData.id = Integer.parseInt(beliefId);
    // 			}
    // 		} else if (field.equals("p") || field.equals("r") || field.equals("s")){
    // 			// Parse position, rotation, or scaling
    // 			double x = Double.parseDouble(fields[i++]);
    // 			double y = Double.parseDouble(fields[i++]);
    // 			double z = Double.parseDouble(fields[i++]);
    // 			if(field.equals("p")){
    // 				// Position
    // 				objData.pos = new double[]{x, y, z, 0, 0, 0};
    // 				objData.bbox_xyzrpy[0] = x;
    // 				objData.bbox_xyzrpy[1] = y;
    // 				objData.bbox_xyzrpy[2] = z;
    // 			} else if(field.equals("r")){
    // 				// Rotation
    // 				objData.bbox_xyzrpy[3] = x;
    // 				objData.bbox_xyzrpy[4] = y;
    // 				objData.bbox_xyzrpy[5] = z;
    // 			} else {
    // 				// Scaling
    // 				objData.bbox_dim = new double[]{x, y, z};
    // 			}
    // 		} else if(field.equals("f")){
    // 			Integer numFlags = Integer.parseInt(fields[i++]);
    // 			ArrayList<categorized_data_t> catDats = new ArrayList<categorized_data_t>();
    // 			for(int j = 0; j < numFlags; j++){
    // 				String flagName = fields[i++];
    // 				String flagVal = fields[i++];
    // 				categorized_data_t catDat = parseFlag(flagName, flagVal);
    // 				if(catDat != null){
    // 					catDats.add(catDat);
    // 				}
    // 			}
    // 			objData.cat_dat = catDats.toArray(new categorized_data_t[catDats.size()]);
    // 			objData.num_cat = objData.cat_dat.length;
    // 		}
    // 	}

    // 	return objData;
    // }

    // public categorized_data_t parseFlag(String flagName, String flagValue){
    // 	categorized_data_t catDat = new categorized_data_t();
    // 	catDat.cat = new category_t();
    // 	Integer catId = PerceptualProperty.getPropertyID(flagName);
    // 	if(catId == null){
    // 		return null;
    // 	}
    // 	catDat.cat.cat = catId;
    // 	catDat.label = new String[]{flagValue};
    // 	catDat.confidence = new double[]{1};
    // 	catDat.len = 1;
    // 	return catDat;
    // }

    /**********************************************************
     * Methods to update working memroy
     **********************************************************/

    private boolean added = false;

    public boolean isAdded(){
    	return added;
    }

    public synchronized void addToWM(Identifier parentId){
    	if(added){
    		removeFromWM();
    	}
		objectsId = parentId.CreateIdWME("objects");
		for(WorldObject obj : objects.values()){
			obj.addToWM(objectsId);
		}

    	StringBuilder svsCommands = new StringBuilder();
        svsCommands.append("add eye world b .01 p 0 0 0\n");
        soarAgent.getAgent().SendSVSInput(svsCommands.toString());

    	added = true;
    }

    public synchronized void updateWM(){
    	if(!added || !needsUpdate){
    		return;
    	}

    	for(WorldObject obj : objects.values()){
    		if(obj.isAdded()){
    			obj.updateWM();
    		} else {
    			obj.addToWM(objectsId);
    		}
    		if(objsToRemove.contains(obj)){
    			objsToRemove.remove(obj);
    		}
    	}

    	for(WorldObject obj : objsToRemove){
    		obj.removeFromWM();
    	}
    	objsToRemove.clear();

    	StringBuilder svsCommands = new StringBuilder();
        svsCommands.append(SVSCommands.changePos("eye", eyePos));
        soarAgent.getAgent().SendSVSInput(svsCommands.toString());

        //sendObservation();

        needsUpdate = false;
    }

    public synchronized void removeFromWM(){
    	if(!added){
    		return;
    	}

    	StringBuilder svsCommands = new StringBuilder();
        svsCommands.append("delete eye\n");
        soarAgent.getAgent().SendSVSInput(svsCommands.toString());

    	added = false;
    }
}
