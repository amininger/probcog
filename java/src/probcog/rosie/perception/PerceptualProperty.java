package probcog.rosie.perception;

import java.util.*;

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
public class PerceptualProperty implements ISoarObject
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

    public PerceptualProperty(categorized_data_t catDat){
    	name = getPropertyName(catDat.cat.cat);
    	type = getPropertyType(catDat.cat.cat);
    	values = new HashMap<String, FloatWME>();
    	featureVal = new FloatWME("feature-val", 0.0);
    	if(type.equals(MEASURABLE_TYPE) && catDat.features.length > 0){
    		featureVal.setValue(catDat.features[0]);
    	}
    	wmesToRemove = new HashSet<FloatWME>();
    }
    
    public String getPropertyName(){
    	return name;
    }
    
    public void updateProperty(HashMap<String, Double> valueInfo){
    	HashSet<String> valuesToRemove = new HashSet<String>(values.keySet());

    	for(Map.Entry<String, Double> e : valueInfo.entrySet()){
    		String valueName = e.getKey();
    		Double conf = e.getValue();
    		
    		if(values.containsKey(valueName)){
    			valuesToRemove.remove(valueName);
    			values.put(valueName, new FloatWME(valueName, conf));
    		} else {
    			values.get(valueName).setValue(conf);
    		}
    	}
    	
    	for(String valueName : valuesToRemove){
    		wmesToRemove.add(values.get(valueName));
    		values.remove(valueName);
    	}
    	gotUpdate = true;
    }
    
    public void updateProperty(categorized_data_t catDat){
    	HashSet<String> valuesToRemove = new HashSet<String>(values.keySet());

    	for(int i = 0; i < catDat.len; i++){
    		String valueName = catDat.label[i];
    		Double conf = catDat.confidence[i];
    		
    		if(values.containsKey(valueName)){
    			valuesToRemove.remove(valueName);
    			values.get(valueName).setValue(conf);
    		} else {
    			values.put(valueName, new FloatWME(valueName, conf));
    		}
    	}
    	
    	for(String valueName : valuesToRemove){
    		wmesToRemove.add(values.get(valueName));
    		values.remove(valueName);
    	}
	    	
    	// Update feature-val
    	if(type.equals(MEASURABLE_TYPE)){
    		featureVal.setValue(catDat.features[0]);
    	}
    	gotUpdate = true;
    }
    
    public categorized_data_t getCatDat(){
    	return getCatDat(name, values);
    }
    
    public static categorized_data_t getCatDat(String propName, HashMap<String, FloatWME> values){
    	categorized_data_t catDat = new categorized_data_t();
		catDat.cat = new category_t();
		catDat.cat.cat = PerceptualProperty.getPropertyID(propName);
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
				wme.addToWM(propId);
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
