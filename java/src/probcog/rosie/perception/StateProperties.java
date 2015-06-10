package probcog.rosie.perception;

import java.util.HashMap;
import java.util.HashSet;

import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.StringWME;

import sml.Identifier;
import sml.StringElement;

public class StateProperties implements ISoarObject{
	private HashMap<String, StringWME> stateProperties;
	private HashSet<StringWME> wmesToRemove;
	
	private boolean gotUpdate = false;
	
    public StateProperties(){
    	stateProperties = new HashMap<String, StringWME>();
    	wmesToRemove = new HashSet<StringWME>();
    }

    public String getProperty(String propName){
    	if(stateProperties.containsKey(propName)){
    		return stateProperties.get(propName).getValue();
    	} else {
    		return null;
    	}
    }   
    
    public void updateProperties(String[] stateInfo){
		HashSet<String> propsToRemove = new HashSet<String>(stateProperties.keySet());
		
    	for(String nameValPair : stateInfo){
    		String[] nameValSplit = nameValPair.split("=");
    		if(nameValSplit.length < 2){
    			continue;
    		}
    		String propName = nameValSplit[0].toLowerCase();
    		String propVal = nameValSplit[1].toLowerCase();
    		
    		if(stateProperties.containsKey(propName)){
    			propsToRemove.remove(propName);
    			stateProperties.get(propName).setValue(propVal);
    		} else {
    			stateProperties.put(propName, new StringWME(propName, propVal));
    		}
    	}
    	
    	for(String propName : propsToRemove){
    		wmesToRemove.add(stateProperties.get(propName));
    		stateProperties.remove(propName);
    	}
    	gotUpdate = true;    
    }
    
    /**********************************************************
     * Methods for managing working memory
     **********************************************************/
    
    private Identifier parentId = null;
    private boolean added = false;
    
    public boolean isAdded(){
    	return added;
    }
    
    public void addToWM(Identifier parentId){
    	if(added){
    		removeFromWM();
    	}
    	this.parentId = parentId;
    	for(StringWME wme : stateProperties.values()){
    		wme.addToWM(parentId);
    	}
    	added = true;
    }
    
    public void updateWM(){
    	if(!added || !gotUpdate){
    		return;
    	}
    	for(StringWME wme : stateProperties.values()){
    		if(!wme.isAdded()){
    			wme.addToWM(parentId);
    		} else {
    			wme.updateWM();
    		}
    	}

    	for(StringWME wme : wmesToRemove){
    		wme.removeFromWM();
    	}
    	wmesToRemove.clear();
    	
    	gotUpdate = false;
    }
    
    public void removeFromWM(){
    	if(!added){
    		return;
    	}
    	parentId = null;
    	for(StringWME wme : stateProperties.values()){
    		wme.removeFromWM();
    	}
    	added = false;
    }
}
