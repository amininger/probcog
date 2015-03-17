package probcog.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import probcog.classify.Classifications;
import probcog.classify.Features;
import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.categorized_data_t;
import probcog.lcmtypes.category_t;
import probcog.perception.Obj;
import probcog.util.BoundingBox;
import sun.security.util.Length;
import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisObject;

public abstract class SimObjectPC implements SimObject, ISimStateful
{
	protected double[] xyzrpy;
	protected double[] lenxyz;

	protected Color  color = Color.gray;
	protected double scale = 1.0;
    
    protected HashMap<String, String[]> possibleStates;
    protected HashMap<String, String> currentState;
    
    protected boolean visible = true;
    
    protected int id;
    
    protected HashMap<FeatureCategory, String> simClassifications;

    public SimObjectPC(SimWorld sw)
    {
    	possibleStates = new HashMap<String, String[]>();
    	currentState = new HashMap<String, String>();
    	simClassifications = new HashMap<FeatureCategory, String>();
    	xyzrpy = new double[]{0.0, 0.0, 0.0, 0.0, 0.0, 0.0};
    	lenxyz = new double[]{1.0, 1.0, 1.0};
    	id = -1;
    }
    
    public int getID(){
    	return id;
    }
    public void setID(int id){
    	this.id = id;
    }

    public double[][] getPose()
    {
    	return LinAlg.xyzrpyToMatrix(xyzrpy);
    }

    public void setPose(double T[][])
    {
    	this.xyzrpy = LinAlg.matrixToXyzrpy(T);
    }
    
    public boolean getVisible(){
    	return visible;
    }
    public void setVisible(boolean visible){
    	this.visible = visible;
    }
    
    public double[] getScaledDims(){
    	return new double[]{scale * lenxyz[0], scale * lenxyz[1], scale * lenxyz[2]};
    }
    
    public Shape getShape(){
    	return new BoxShape(scale*lenxyz[0], scale*lenxyz[1], scale*lenxyz[2]);
    }

    public abstract VisObject getVisObject();
    
    public Obj getObj()
    {
    	if(id == Obj.NULL_ID){
    		id = Obj.nextID();
    	}
        
    	Obj obj = new Obj(id);
    	
    	obj.setPose(xyzrpy);
    	obj.setCentroid(LinAlg.copy(xyzrpy, 3));
    	obj.setBoundingBox(new BoundingBox(getScaledDims(), xyzrpy));
    	
    	obj.setVisObject(getVisObject());
    	obj.setShape(getShape());
    	obj.setSourceSimObject(this);
        
        obj.setStates(this.getCurrentState());
        for(Map.Entry<FeatureCategory, String> e : simClassifications.entrySet()){
        	obj.setLabel(e.getKey(), e.getValue());
        }

        return obj;
    }


	@Override
	public String getState(String stateName) {
		return currentState.get(stateName.toLowerCase());
	}

	@Override
	public void setState(String stateName, String stateVal) {
		String[] validValues = possibleStates.get(stateName.toLowerCase());
		if(validValues == null){
			return;
		}
		stateVal = stateVal.toLowerCase();
		for(String val : validValues){
			if(val.equals(stateVal)){
				currentState.put(stateName, stateVal);
				return;
			}
		}
	}

	@Override
	public String[][] getCurrentState() {
		String[][] states = new String[currentState.size()][2];
		int i = 0;
		for(Map.Entry<String, String> state : currentState.entrySet()){
			states[i][0] = state.getKey();
			states[i][1] = state.getValue();
			i++;
		}
		return states;
	}
	
	public HashMap<FeatureCategory, String> getSimClassifications(){
		return simClassifications;
	}
	
	public void addNewState(String stateName, String[] possibleValues){
		if(possibleValues.length == 0){
			return;
		}
		FeatureCategory fc = Features.getFeatureCategory(stateName);
		if(fc != null){
			simClassifications.put(fc, possibleValues[0]);
			return;
		}
		
		stateName = stateName.toLowerCase();
		for(int i = 0; i < possibleValues.length; i++){
			possibleValues[i] = possibleValues[i].toLowerCase();
		}
		possibleStates.put(stateName, possibleValues);
		currentState.put(stateName, possibleValues[0]);
	}
	
	private void addNewState(String stateInfo){
        String[] nameVals = stateInfo.split("=");
        if(nameVals.length >= 2) {
            String[] allowedStates = nameVals[1].split(",");
            addNewState(nameVals[0], allowedStates);
        }
	}
    
    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
    	id = ins.readInt();
    	if(id == -1){
    		id = Obj.nextID();
    	} else {
    		Obj.idAssigned(id);
    	}
    	
    	// 6 doubles for pose information (XYZRPY)
        xyzrpy = ins.readDoubles();
        
        // 1 double for scale
        this.scale = ins.readDouble();

        // 4 doubles for color (RGBA)
        double c[] = ins.readDoubles();
        this.color = new Color((float) c[0], (float) c[1], (float) c[2], (float) c[3]);
        
        // 1 integer for number of states
        int numStates = ins.readInt();
        // Followed by n strings of the form 'name=val1,val2,val3'
        // Example: 'door=open,closed'
        String[] states = new String[numStates];
        for(int i = 0; i < numStates; i++){
            states[i] = ins.readString();
        }
        for(String stateInfo : states) {
        	addNewState(stateInfo);
        }
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeInt(id);
        outs.writeDoubles(xyzrpy);
        outs.writeDouble(scale);
        float f[] = color.getRGBComponents(null);
        outs.writeDoubles(LinAlg.copyDoubles(f));
        outs.writeInt(possibleStates.size());
        for(Map.Entry<String, String[]> state : possibleStates.entrySet()){
        	String curStateVal = currentState.get(state.getKey());
        	StringBuilder stateInfo = new StringBuilder();
        	stateInfo.append(state.getKey() + "=");
        	stateInfo.append(curStateVal);
        	for(String possVal : state.getValue()){
        		if(possVal.equals(curStateVal)){
        			continue;
        		}
        		stateInfo.append("," + possVal);
        	}
        	outs.writeString(stateInfo.toString());
        }
    }

    public void setRunning(boolean b)
    {
    }
}
