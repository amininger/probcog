package soargroup.rosie.perception;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Arrays;

import java.nio.ByteBuffer;

import april.jmat.LinAlg;
import soargroup.mobilesim.lcmtypes.classification_t;
import soargroup.mobilesim.lcmtypes.object_data_t;
import sml.Identifier;
import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.IntWME;
import edu.umich.rosie.soar.SVSCommands;
import edu.umich.rosie.soar.StringWME;

public class WorldObject implements ISoarObject {
	private Integer tagID;
	private IntWME tagWME;
	private StringWME handle;
	private HashMap<String, ObjectProperty> properties;

	private boolean updatePos = true;
	private double[] pos = new double[3];

	private boolean updateRot = true;
	private double[] rot = new double[3];

//	private boolean updateScale = true;
	private double[] scale = new double[3];

	private StringBuilder svsCommands;

	private boolean changed = false;

	public WorldObject(Integer tagID, double[] pos, double[] scale, HashMap<String, String> classifications){
		this.tagID = tagID;
		this.pos = pos;
		this.scale = scale;
		this.handle = new StringWME("object-handle", tagID.toString());
		this.tagWME = new IntWME("tag-id", (long)tagID);
		
		this.properties = new HashMap<String, ObjectProperty>();
		for(Map.Entry<String, String> e : classifications.entrySet()){
			ObjectProperty p = new ObjectProperty(e.getKey(), ObjectProperty.VISUAL_TYPE, e.getValue());
			this.properties.put(e.getKey(), p);
		}
		svsCommands = new StringBuilder();
	}

	public Integer getTagID(){
		return tagID;
	}
	
	public void setPos(double[] pos){
		this.pos = pos;
		updatePos = true;
	}

	public double[] getPos(){
		return pos;
	}

	public synchronized void addProperty(String name, String value){
		if(properties.containsKey(name)){
			return;
		}
		ObjectProperty p = new ObjectProperty(name, ObjectProperty.VISUAL_TYPE, value);
		properties.put(name, p);
		changed = true;
	}

	public String getHandle(){
		return handle.getValue();
	}

	public synchronized void setHandle(String handle){
		this.handle.setValue(handle);
		changed = true;
	}

	public synchronized void update(double[] robotPose, byte[] data){
        double[] pose = new double[7];
        ByteBuffer bb = ByteBuffer.wrap(data);
        for (int i = 0; i < pose.length; i++) {
            pose[i] = bb.getDouble();
        }

        double[] rpy = LinAlg.quatToRollPitchYaw(Arrays.copyOfRange(pose, 3, 7));

        double yaw = robotPose[5];
        double ct = Math.cos(yaw);
        double st = Math.sin(yaw);
        double x = robotPose[0] + ct * pose[0] - st*pose[1];
        double y = robotPose[1] + st * pose[0] + ct*pose[1];
        double z = pose[2] + robotPose[2];
        double[] pos = new double[]{ x, y, z };

		for(int d = 0; d < 3; d++){
			// Only update pos if it has changed by a significant amount
			if(Math.abs(this.pos[d] - pos[d]) > 0.2){
				this.pos[d] = pos[d];
				updatePos = true;
			}
    }
		//for(int d = 0; d < 3; d++){
		//	// Only update rot if it has changed by a significant amount
		//	if(Math.abs(rot[d] - rpy[d]) > 0.05){
		//		rot[d] = rpy[d];
		//		updateRot = true;
		//	}
//			// Only update scale if it was changed by a significant amount
//			if(Math.abs(scale[d] - newData.lenxyz[d]) > 0.01){
//				scale[d] = newData.lenxyz[d];
//				updateScale = true;
//			}
	}


	public synchronized String getSVSCommands(){
		String commands = svsCommands.toString();
		svsCommands = new StringBuilder();
		return commands;
	}

	 /******************************************************************
     * Methods for Modifying Working Memory
     *****************************************************************/
	private Identifier rootID = null;
    private boolean added = false;

    public boolean isAdded(){
    	return added;
    }

    public synchronized void addToWM(Identifier parentID){
    	if(added){
    		removeFromWM();
    	}

    	rootID = parentID.CreateIdWME("object");
    	handle.addToWM(rootID);
    	tagWME.addToWM(rootID);
    	for (Map.Entry<String, ObjectProperty> e : properties.entrySet()){
    		e.getValue().addToWM(rootID);
    	}

//    	double[] defaultRot = new double[]{ 0.0, 0.0, 0.0 };
//    	double[] defaultScale = new double[]{ .5, .5, .5 };
    	svsCommands.append(SVSCommands.addBox(handle.getValue(), pos, rot, scale));
    	svsCommands.append(SVSCommands.addTag(handle.getValue(), "object-source", "perception"));
//    	for (ObjectProperty prop : properties.values()){
//    		svsCommands.append(SVSCommands.addTag(handle.getValue(), prop.getPropertyName(), wme.getAttribute(), wme.getValue()));
//    	}

        added = true;
    	changed = false;
    }

    public synchronized void updateWM(){
    	if(!added){
    		return;
    	}
        // Update the pose in SVS
    	if(updatePos){
   			svsCommands.append(SVSCommands.changePos(handle.getValue(), pos));
   			updatePos = false;
    	}
   		if(updateRot){
   			svsCommands.append(SVSCommands.changeRot(handle.getValue(), rot));
   			updateRot = false;
   		}
//   		if(updateScale){
//   			svsCommands.append(SVSCommands.changeScale(handle.getValue(), scale));
//   			updateScale = false;
//   		}

    	if(changed){
    		handle.updateWM();

    		for(ObjectProperty prop : properties.values()){
    			if(!prop.isAdded()){
    				prop.addToWM(rootID);
    				//svsCommands.append(SVSCommands.addTag(handle.getValue(), wme.getAttribute(), wme.getValue()));
    			}
    		}

    		changed = false;
    	}
    }

    public synchronized void removeFromWM(){
    	if(!added){
    		return;
    	}
        // Remove the object from the input link
    	for(ObjectProperty prop : properties.values()){
    		prop.removeFromWM();
    	}
    	handle.removeFromWM();
    	tagWME.removeFromWM();
    	rootID.DestroyWME();
    	rootID = null;

    	svsCommands.append(SVSCommands.delete(handle.getValue()));
    	added = false;
    }
}
