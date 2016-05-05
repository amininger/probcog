package probcog.rosie.perception;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.Arrays;

import april.jmat.LinAlg;
import probcog.lcmtypes.classification_t;
import probcog.lcmtypes.object_data_t;
import sml.Identifier;
import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.IntWME;
import edu.umich.rosie.soar.SVSCommands;
import edu.umich.rosie.soar.StringWME;

public class WorldObject implements ISoarObject {
	private Integer tagID;
	private IntWME tagWME;
	private StringWME handle;
	private HashMap<String, StringWME> classifications;
	
	private boolean updatePos = true;
	private double[] pos = new double[3];
	
	private boolean updateRot = true;
	private double[] rot = new double[3];
	
//	private boolean updateScale = true;
	private double[] scale = new double[3];
	
	private StringBuilder svsCommands;
	
	private boolean changed = false;
	
	public WorldObject(Integer tagID, double[] scale, HashMap<String, String> classifications){
		this.tagID = tagID;
		this.scale = scale;
		this.handle = new StringWME("handle", "none");
		this.tagWME = new IntWME("tag-id", (long)tagID);

		this.classifications = new HashMap<String, StringWME>();
		for(Map.Entry<String, String> e : classifications.entrySet()){
			addClassification(e.getKey(), e.getValue());
		}
		svsCommands = new StringBuilder();
	}
	
	public Integer getTagID(){
		return tagID;
	}
	
	public double[] getPos(){
		return pos;
	}
	
	public synchronized void addClassification(String name, String value){
		if(classifications.containsKey(name)){
			return;
		}
		classifications.put(name, new StringWME(name, value));
		changed = true;
	}
	
	public String getHandle(){
		return handle.getValue();
	}
	
	public synchronized void setHandle(String handle){
		this.handle.setValue(handle);
		changed = true;
	}

  public static double[] toDoubleArray(byte[] byteArray){
    int times = Double.SIZE / Byte.SIZE;
    double[] doubles = new double[byteArray.length / times];
    for(int i=0;i<doubles.length;i++){
      doubles[i] = ByteBuffer.wrap(byteArray, i*times, times).getDouble();
    }
    return doubles;
  }
	
	public synchronized void update(byte[] data){
    System.out.println("============");
    double[] pose = toDoubleArray(data);
    for(double d : pose){
      System.out.println(d);
    }
    pose = new double[]{ 0, 0, 0, 0, 1, 0, 0 };

		for(int d = 0; d < 3; d++){
			// Only update pos if it has changed by a significant amount
			if(Math.abs(this.pos[d] - pose[d]) > 0.02){
				this.pos[d] = pose[d];
				updatePos = true;
			}
    }
    double[] rpy = LinAlg.quatToRollPitchYaw(Arrays.copyOfRange(pose, 3, 7));
		for(int d = 0; d < 3; d++){
			// Only update rot if it has changed by a significant amount
			if(Math.abs(rot[d] - rpy[d]) > 0.05){
				rot[d] = rpy[d];
				updateRot = true;
			}
//			// Only update scale if it was changed by a significant amount
//			if(Math.abs(scale[d] - newData.lenxyz[d]) > 0.01){
//				scale[d] = newData.lenxyz[d];
//				updateScale = true;
//			}
		}
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
	private Identifier classificationsID = null;
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
    	classificationsID = rootID.CreateIdWME("classifications");
    	for (Map.Entry<String, StringWME> e : classifications.entrySet()){
    		e.getValue().addToWM(classificationsID);
    	}
    	
//    	double[] defaultRot = new double[]{ 0.0, 0.0, 0.0 };
//    	double[] defaultScale = new double[]{ .5, .5, .5 };
    	svsCommands.append(SVSCommands.addBox(handle.getValue(), pos, rot, scale));
    	svsCommands.append(SVSCommands.addTag(handle.getValue(), "object-source", "perception"));
    	for (StringWME wme : classifications.values()){
    		svsCommands.append(SVSCommands.addTag(handle.getValue(), wme.getAttribute(), wme.getValue()));
    	}

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

    		for(StringWME wme : classifications.values()){
    			if(!wme.isAdded()){
    				wme.addToWM(classificationsID);
    				svsCommands.append(SVSCommands.addTag(handle.getValue(), wme.getAttribute(), wme.getValue()));
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
    	for(StringWME wme : classifications.values()){
    		wme.removeFromWM();
    	}
    	classificationsID = null;
    	handle.removeFromWM();
    	tagWME.removeFromWM();
    	rootID.DestroyWME();
    	rootID = null;
    	
    	svsCommands.append(SVSCommands.delete(handle.getValue()));
    	added = false;
    }
}
