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

import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.messages.*;
import edu.wpi.rail.jrosbridge.callback.*;
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

    private Ros ros;
    private Topic soarObjs;

    public WorldModel(SoarAgent soarAgent){
    	this.soarAgent = soarAgent;

        objects = new HashMap<Integer, WorldObject>();
        objsToRemove = new HashSet<WorldObject>();
        objectLinks = new HashMap<Integer, Integer>();

        eyePos = new double[6];

        ros = new Ros();
        ros.connect();

        if (ros.isConnected()) {
            System.out.println("WorldModel connected to rosbridge server.");
        }
        else {
            System.out.println("WorldModel NOT CONNECTED TO ROSBRIDGE");
        }

        soarObjs = new Topic(ros,
                             "/rosie_soar_obj",
                             "rosie_msgs/SoarObjects",
                             500);
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

    	ArrayList<ObjectData> objData = new ArrayList<ObjectData>();

    	for(String sourceHandle : sourceHandles){
    		Integer sHandle = Integer.parseInt(sourceHandle);
    		objectLinks.put(sHandle, dHandle);
    		if(objects.containsKey(sHandle)){
    			WorldObject wobj = objects.get(sHandle);
    			objData.addAll(wobj.getLastDatas());
    			objsToRemove.add(wobj);
    			objects.remove(sHandle);
    		}
    	}

        if(objData.size() > 0){
        	if(objects.containsKey(dHandle)){
        		WorldObject dObj = objects.get(dHandle);
        		objData.addAll(dObj.getLastDatas());
    			dObj.update(objData);
    		} else {
    			WorldObject object = new WorldObject(this, dHandle, objData);
    			objects.put(dHandle, object);
    		}
        }
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

    	// For each object, either update existing or create if new
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

    	// Remove all stale objects from WM
        for(Integer handle : staleObjects){
        	WorldObject object = objects.get(handle);
        	objsToRemove.add(object);
        	objects.remove(handle);
        }

        needsUpdate = true;
    }

    private void sendObservation(){
    	ArrayList<ObjectData> objDatas = new ArrayList<ObjectData>();

    	String[] beliefObjects = soarAgent.getAgent().SVSQuery("objs-with-flag object-source belief\n").split(" ");
    	for(int i = 2; i < beliefObjects.length; i++){
    		String beliefId = beliefObjects[i].trim();
    		if(beliefId.length() == 0){
    			continue;
    		}
    		String obj = soarAgent.getAgent().SVSQuery("obj-info " + beliefId);
    		objDatas.add(parseObject(obj));
    	}

        StringBuilder outgoingObs = new StringBuilder();
        outgoingObs.append("{");

        long utime = TimeUtil.utime();
        outgoingObs.append("\"header\": {\"stamp\": " + utime + "}, ");
        outgoingObs.append("\"objects\": [");

        int count = 0;
        for (ObjectData od : objDatas) {
            if (count > 0) outgoingObs.append(",");
            count++;
            outgoingObs.append(od.toJsonString());
        }
        outgoingObs.append("]}");

        //System.out.println(outgoingObs.toString());
        Message m = new Message(outgoingObs.toString());
        soarObjs.publish(m);
    }

    public ObjectData parseObject(String objInfo){
    	ObjectData objData = new ObjectData();

    	String[] fields = objInfo.split(" ");
    	int i = 0;
    	while(i < fields.length){
    		String field = fields[i++];
    		if(field.equals("o")){
    			// Parse ID: Should be in format bel-#
    			String beliefId = fields[i++];
    			Integer index = beliefId.indexOf("bel-");
    			if(index == 0){
    				objData.setID(Integer.parseInt(beliefId.substring(4)));
    			} else {
    				objData.setID(Integer.parseInt(beliefId));
    			}
    		} else if (field.equals("p") || field.equals("r") || field.equals("s")){
    			// Parse position, rotation, or scaling
    			double x = Double.parseDouble(fields[i++]);
    			double y = Double.parseDouble(fields[i++]);
    			double z = Double.parseDouble(fields[i++]);
    			if(field.equals("p")){
    				// Position
    				objData.setPos(new double[]{x, y, z, 0, 0, 0});
    				objData.setBBoxPos(0, x);
    				objData.setBBoxPos(1, y);
    				objData.setBBoxPos(2, z);
    			} else if(field.equals("r")){
    				// Rotation
    				objData.setBBoxPos(3, x);
    				objData.setBBoxPos(4, y);
    				objData.setBBoxPos(5, z);
    			} else {
    				// Scaling
    				objData.setBBoxDim(new double[]{x, y, z});
    			}
    		} else if(field.equals("f")){
    			Integer numFlags = Integer.parseInt(fields[i++]);
    			ArrayList<CategorizedData> catDats = new ArrayList<CategorizedData>();
    			for(int j = 0; j < numFlags; j++){
    				String flagName = fields[i++];
    				String flagVal = fields[i++];
    				CategorizedData catDat = parseFlag(flagName, flagVal);
    				if(catDat != null){
    					catDats.add(catDat);
    				}
    			}
    	 		objData.setCatDat(catDats);
    	 	}
    	}

    	return objData;
    }

    public CategorizedData parseFlag(String flagName, String flagValue){
        CategorizedData.CategoryType catId = PerceptualProperty.getPropertyID(flagName);
    	if(catId == null){
    		return null;
    	}

     	CategorizedData catDat = new CategorizedData(catId);
        catDat.addLabel(flagValue, 1);
    	return catDat;
    }

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

        sendObservation();

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
