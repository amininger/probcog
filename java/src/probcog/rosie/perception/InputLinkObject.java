package probcog.rosie.perception;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import april.jmat.LinAlg;
import probcog.lcmtypes.classification_t;
import probcog.lcmtypes.object_data_t;
import sml.Identifier;
import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.SVSCommands;
import edu.umich.rosie.soar.StringWME;

public class InputLinkObject implements ISoarObject {
	private StringWME handle;
	private HashMap<String, StringWME> classifications;
	
	private boolean updatePos = true;
	private double[] pos = new double[3];
	
	private boolean updateRot = true;
	private double[] rot = new double[3];
	
	private boolean updateScale = true;
	private double[] scale = new double[3];

	
	private StringBuilder svsCommands;
	
	private boolean changed = false;
	private HashSet<StringWME> wmesToRemove;
	
	
	public InputLinkObject(object_data_t objectData){
		handle = new StringWME("handle", objectData.id);

		wmesToRemove = new HashSet<StringWME>();
		svsCommands = new StringBuilder();
		
		update(objectData);
	}
	
	public synchronized void update(object_data_t newData){
		for(int d = 0; d < 3; d++){
			// Only update pos if it has changed by a significant amount
			if(Math.abs(pos[d] - newData.xyzrpy[d]) > 0.02){
				pos[d] = newData.xyzrpy[d];
				updatePos = true;
			}
			// Only update rot if it has changed by a significant amount
			if(Math.abs(rot[d] - newData.xyzrpy[3+d]) > 0.05){
				rot[d] = newData.xyzrpy[3+d];
				updateRot = true;
			}
			// Only update scale if it was changed by a significant amount
			if(Math.abs(scale[d] - newData.lenxyz[3]) > 0.01){
				scale[d] = newData.lenxyz[d];
				updateScale = true;
			}
		}
		
		HashSet<String> missingClassifications = new HashSet<String>();
		missingClassifications.addAll(classifications.keySet());

		for (classification_t classification : newData.classifications){
			StringWME wme = classifications.get(classification.category);
			if (wme == null){
				wme = new StringWME(classification.category, classification.name);
				classifications.put(classification.category, wme);
			} else {
				missingClassifications.remove(classification.category);
			}
		}
		
		for(String cat : missingClassifications){
			wmesToRemove.add(classifications.get(cat));
			classifications.remove(cat);
		}
		
		changed = true;
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
    	classificationsID = rootID.CreateIdWME("classifications");
    	for (Map.Entry<String, StringWME> e : classifications.entrySet()){
    		e.getValue().addToWM(classificationsID);
    	}
    	
    	svsCommands.append(SVSCommands.addBox(handle.getValue(), pos, rot, scale));
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
        // Update the pose on the input link
    	if(changed){
    		if(updatePos){
    			svsCommands.append(SVSCommands.changePos(handle.getValue(), pos));
    			updatePos = false;
    		}
    		if(updateRot){
    			svsCommands.append(SVSCommands.changeRot(handle.getValue(), rot));
    			updateRot = false;
    		}
    		if(updateScale){
    			svsCommands.append(SVSCommands.changeScale(handle.getValue(), scale));
    			updateScale = false;
    		}

    		for(StringWME wme : classifications.values()){
    			if(wme.isAdded()){
    				wme.updateWM();
    				svsCommands.append(SVSCommands.addTag(handle.getValue(), wme.getAttribute(), wme.getValue()));
    			} else {
    				wme.addToWM(classificationsID);
    				svsCommands.append(SVSCommands.changeTag(handle.getValue(), wme.getAttribute(), wme.getValue()));
    			}
    		}
    		for(StringWME wme : wmesToRemove){
    			wme.removeFromWM();
    			svsCommands.append(SVSCommands.deleteTag(handle.getValue(), wme.getAttribute()));
    		}
    		wmesToRemove.clear();
    		
    		changed = false;
    	}
    }
    
    public synchronized void removeFromWM(){
    	if(!added){
    		return;
    	}
        // Remove the object from the input link
    	wmesToRemove.clear();
    	for(StringWME wme : classifications.values()){
    		wme.removeFromWM();
    	}
    	classificationsID = null;
    	handle.removeFromWM();
    	rootID.DestroyWME();
    	rootID = null;
    	
    	svsCommands.append(SVSCommands.delete(handle.getValue()));
    	added = false;
    }
}
