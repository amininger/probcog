package probcog.rosie.perception;

import java.util.*;
import java.util.HashMap;

import edu.umich.rosie.soar.FloatWME;
import edu.umich.rosie.soar.ISoarObject;

import probcog.lcmtypes.*;
import sml.*;

/**
 * A category for each object, which contains several possible labels and their confidences
 * 
 * @author mininger
 * 
 */
public class ObjectProperty implements ISoarObject
{   
	protected static HashMap<Integer, String> propertyNames = null;
	
	public static String getPropertyName(Integer propertyID){
		if(propertyNames == null){
			propertyNames = new HashMap<Integer, String>();
			propertyNames.put(category_t.CAT_COLOR, "color");
			propertyNames.put(category_t.CAT_SHAPE, "shape");
			propertyNames.put(category_t.CAT_SIZE, "size");
			propertyNames.put(category_t.CAT_LOCATION, "name");
			propertyNames.put(category_t.CAT_WEIGHT, "weight");
			propertyNames.put(category_t.CAT_TEMPERATURE, "temperature");
		}
		return propertyNames.get(propertyID);
	}
	
	public static final String VISUAL_TYPE = "visual";
	public static final String LINGUISTIC_TYPE = "linguistic";
	public static final String MEASURABLE_TYPE = "measurable";
	public static final String STATE_TYPE = "state";
	protected static HashMap<Integer, String> propertyTypes = null;
	public static String getPropertyType(Integer propertyCategory){
		if(propertyTypes == null){
			propertyTypes = new HashMap<Integer, String>();
			propertyTypes.put(category_t.CAT_COLOR, VISUAL_TYPE);
			propertyTypes.put(category_t.CAT_SHAPE, VISUAL_TYPE);
			propertyTypes.put(category_t.CAT_SIZE, VISUAL_TYPE);
			propertyTypes.put(category_t.CAT_LOCATION, LINGUISTIC_TYPE);
			propertyTypes.put(category_t.CAT_WEIGHT, MEASURABLE_TYPE);
			propertyTypes.put(category_t.CAT_TEMPERATURE, MEASURABLE_TYPE);
		}
		return propertyTypes.get(propertyCategory);
	}
	
	public static Integer getPropertyID(String propertyName){
		if(propertyName.equals("color")){
			return category_t.CAT_COLOR;
		} else if(propertyName.equals("shape")){
			return category_t.CAT_SHAPE;
		} else if(propertyName.equals("size")){
			return category_t.CAT_SIZE;
		} else if(propertyName.equals("name")){
			return category_t.CAT_LOCATION;
		} else if(propertyName.equals("weight")){
			return category_t.CAT_WEIGHT;
		} else if(propertyName.equals("temperature")){
			return category_t.CAT_TEMPERATURE;
		} else {
			return null;
		}
	}
	
    
    protected String name;
    protected String type;
    protected HashMap<String, FloatWME> values;
    protected HashSet<FloatWME> wmesToRemove;
    protected FloatWME featureVal;
    
    private boolean gotUpdate = false;

    public ObjectProperty(categorized_data_t catDat){
    	name = getPropertyName(catDat.cat.cat);
    	type = getPropertyType(catDat.cat.cat);
    	values = new HashMap<String, FloatWME>();
    	featureVal = new FloatWME("feature-val", 0.0);
    	if(type.equals(MEASURABLE_TYPE) && catDat.features.length > 0){
    		featureVal.setValue(catDat.features[0]);
    	}
    	wmesToRemove = new HashSet<FloatWME>();
    }
    
    public ObjectProperty(String propName, String type, String propValue){
    	this.name = propName;
    	this.type = type;
    	this.values = new HashMap<String, FloatWME>();
    	this.values.put(propValue, new FloatWME(propValue, 1.0));
    	this.featureVal = new FloatWME("feature-val", 0.0);
    	this.wmesToRemove = new HashSet<FloatWME>();
    }
    
    public String getPropertyName(){
    	return name;
    }
    
    public synchronized void updateProperty(HashMap<String, Double> valueInfo){
    	HashSet<String> valuesToRemove = new HashSet<String>(values.keySet());

    	for(Map.Entry<String, Double> e : valueInfo.entrySet()){
    		String valueName = e.getKey();
    		Double conf = e.getValue();
    		
    		if(values.containsKey(valueName)){
    			values.get(valueName).setValue(conf);
    			valuesToRemove.remove(valueName);
    		} else {
    			values.put(valueName, new FloatWME(valueName, conf));
    		}
    	}
    	
    	for(String valueName : valuesToRemove){
    		wmesToRemove.add(values.get(valueName));
    		values.remove(valueName);
    	}
    	gotUpdate = true;
    }
    
    public synchronized void updateProperty(String newValue){
    	String curValue = null;
    	// Should only be 1 entry in the map
    	for(Map.Entry<String, FloatWME> e : this.values.entrySet()){
    		curValue = e.getKey();
    	}
    	if (!curValue.equals(newValue)){
    		wmesToRemove.add(this.values.get(curValue));
    		this.values.remove(curValue);
    		this.values.put(newValue, new FloatWME(newValue, 1.0));
    	}
    	gotUpdate = true;
    }
    
    public void updateProperty(categorized_data_t catDat){
    	HashMap<String, Double> valueInfo = new HashMap<String, Double>();
    	for(int i = 0; i < catDat.len; i++){
    		valueInfo.put(catDat.label[i], catDat.confidence[i]);
    	}
    	updateProperty(valueInfo);
    }
    
    public categorized_data_t getCatDat(){
    	return getCatDat(name, values);
    }
    
    public static categorized_data_t getCatDat(String propName, HashMap<String, FloatWME> values){
    	categorized_data_t catDat = new categorized_data_t();
		catDat.cat = new category_t();
		catDat.cat.cat = ObjectProperty.getPropertyID(propName);
		catDat.len = values.size();
		catDat.label = new String[catDat.len];
		catDat.confidence = new double[catDat.len];
		
		int i = 0;
		for(Map.Entry<String, FloatWME> val : values.entrySet()){
			catDat.label[i] = val.getKey();
			catDat.confidence[i] = val.getValue().getValue();
			i++;
		}
		
		return catDat;
    }

    /**************************************************************
     * Methods for adding to working memory
     **************************************************************/

	protected Identifier propId = null;
    protected Identifier valuesId = null;
    private boolean added = false;
    
    public boolean isAdded(){
    	return added;
    }
    
    public void addToWM(Identifier parentId){
    	if(added){
    		removeFromWM();
    	}

		propId = parentId.CreateIdWME("property");
		propId.CreateStringWME("property-handle", name);
		propId.CreateStringWME("type", type);
		valuesId = propId.CreateIdWME("values");
		for(FloatWME wme : values.values()){
			wme.addToWM(valuesId);
		}
		if(type.equals(MEASURABLE_TYPE)){
			featureVal.addToWM(propId);
		}
		added = true;
    }
	
	public void updateWM(){
		if(!added || !gotUpdate){
			return;
		}
		
		for(FloatWME wme : values.values()){
			if(!wme.isAdded()){
				wme.addToWM(valuesId);
			} else {
				wme.updateWM();
			}
		}
		
		for(FloatWME wme : wmesToRemove){
			wme.removeFromWM();
		}
		wmesToRemove.clear();
	    	
    	// Update feature-val
    	if(type.equals(MEASURABLE_TYPE)){
    		featureVal.updateWM();
    	}
    	gotUpdate = false;
    }
    
    public void removeFromWM(){
    	if(!added){
    		return;
    	}
    	for(FloatWME wme : values.values()){
    		wme.removeFromWM();
    	}
    	propId.DestroyWME();
    	propId = null;
    	valuesId = null;
    	added = false;
    }
}
