package probcog.rosie.perception;

import java.util.HashMap;
import java.util.HashSet;

import sml.Identifier;
import sml.StringElement;

public class StateProperties implements IInputLinkElement{
	protected class WMStructure implements IInputLinkElement{
		public Identifier propId = null;
		
		public String name;

		public StringElement valueWme;
		public String value;
		private boolean changed = true;
		
		public WMStructure(String name, String value){
			this.name = name;
			this.value = value;
		}
		
		public void setValue(String value){
			if(!this.value.equals(value)){
				changed = true;
				this.value = value;
			}
		}
		public void destroy(){
			if(propId != null){
				propId.DestroyWME();
				propId = null;
				valueWme = null;
			}
		}
		
		public void updateInputLink(Identifier parentIdentifier) {
			if(propId == null){
				propId = parentIdentifier.CreateIdWME("property");
				propId.CreateStringWME("name", name);
				propId.CreateStringWME("type", PerceptualProperty.STATE_TYPE);
				valueWme = propId.CreateStringWME("value", value);
			}
			if(changed){
				valueWme.Update(value);
				changed = false;
			}
		}
	
		public void onInitSoar() {
			destroy();
		}
	}

	private HashMap<String, WMStructure> stateProperties;
	private String[] stateInfo = null;
	
    public StateProperties(){
    	stateProperties = new HashMap<String, WMStructure>();
    }

    public String getProperty(String propName){
    	if(stateProperties.containsKey(propName)){
    		return stateProperties.get(propName).value;
    	} else {
    		return null;
    	}
    }   
    
    public void updateProperties(String[] stateInfo){
    	this.stateInfo = stateInfo;
    }
    
	public void updateInputLink(Identifier parentIdentifier) {
		if(stateInfo == null){
			return;
		}
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
    			stateProperties.put(propName, new WMStructure(propName, propVal));
    		}
    	}
    	
    	for(String propName : propsToRemove){
    		stateProperties.get(propName).destroy();
    		stateProperties.remove(propName);
    	}
    	for(WMStructure struct : stateProperties.values()){
    		struct.updateInputLink(parentIdentifier);
    	}
    	stateInfo = null;
    }
    
    public void destroy(){
    	for(WMStructure propStruct : stateProperties.values()){
    		propStruct.destroy();
    	}
    	stateProperties.clear();
    }

	@Override
	public void onInitSoar() {
		destroy();
	}
}
