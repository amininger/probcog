package probcog.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.List;
import java.util.Map;

import probcog.classify.Classifications;
import probcog.classify.Features;
import probcog.lcmtypes.categorized_data_t;
import probcog.lcmtypes.category_t;
import probcog.perception.Obj;
import april.jmat.LinAlg;
import april.sim.Shape;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisObject;

public abstract class SimObjectPC implements SimObject, ISimStateful
{
	protected double T[][] = LinAlg.identity(4);  // position
	protected Color  color = Color.gray;
	protected double scale = 1.0;
    
    protected HashMap<String, String[]> possibleStates;
    protected HashMap<String, String> currentState;
    
    protected boolean visible = true;
    
    protected int id;
    
    protected List<categorized_data_t> cat_dat = new ArrayList<categorized_data_t>();

    public SimObjectPC(SimWorld sw)
    {
    	possibleStates = new HashMap<String, String[]>();
    	currentState = new HashMap<String, String>();
    	id = Obj.nextID();
    }
    
    public int getID(){
    	return id;
    }
    public void setID(int id){
    	this.id = id;
    }

    public double[][] getPose()
    {
        return LinAlg.copy(T);
    }

    public void setPose(double T[][])
    {
        this.T = LinAlg.copy(T);
    }
    
    public boolean getVisible(){
    	return visible;
    }
    public void setVisible(boolean visible){
    	this.visible = visible;
    }

    public abstract Shape getShape();

    public abstract VisObject getVisObject();


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
	
	public categorized_data_t[] getCategorizedData(){
		categorized_data_t[] cat_dat_a = new categorized_data_t[cat_dat.size()];
		for(int i = 0; i < cat_dat.size(); i++){
			cat_dat_a[i] = cat_dat.get(i);
		}
		return cat_dat_a;
	}
	
	public void addNewState(String stateName, String[] possibleValues){
		if(possibleValues.length == 0){
			return;
		}
		if(stateName.equals("color") || stateName.equals("shape") || stateName.equals("size")){
			categorized_data_t cd = new categorized_data_t();
			cd.cat = new category_t();
			if(stateName.equals("color")){
				cd.cat.cat = category_t.CAT_COLOR;
			} else if(stateName.equals("shape")){
				cd.cat.cat = category_t.CAT_SHAPE;
			} else {
				cd.cat.cat = category_t.CAT_SIZE;
			}
			cd.len = 1;
			cd.confidence = new double[]{1};
			cd.label = new String[]{possibleValues[0]};
			cat_dat.add(cd);
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
    	// 6 doubles for pose information (XYZRPY)
        double xyzrpy[] = ins.readDoubles();
        this.T = LinAlg.xyzrpyToMatrix(xyzrpy);
        
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
        outs.writeDoubles(LinAlg.matrixToXyzrpy(T));
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
