package probcog.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.Map;

import april.jmat.LinAlg;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;

public class SimSteak extends SimBoxPC{
	protected static Color rawColor = new Color(1.0f, .7f, .6f, 1.0f);
	protected static Color cookedColor = new Color(.4f, .2f, .1f, 1.0f);

	protected Boolean isCooked = false;
	protected int temp = 0;
	
	public SimSteak(SimWorld world){
		super(world);
	}
	
    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
    	super.read(ins);
        
        // Boolean for cooked (false = raw)
        this.addNewState("cooked", new String[]{"false", "true"});
		this.setState("cooked", ins.readString());
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	super.write(outs);
        outs.writeString(isCooked ? "true" : "false");
    }
    
    public void setState(String stateName, String stateVal){
    	if(stateName.equals("cooking") && stateVal.equals("true")){
    		if(!isCooked){
    			temp++;
    			if(temp > 60){
    				isCooked = true;
    	        	this.color = cookedColor;
    	        	super.setState("cooked", "true");
    			}
    		}
			return;
    	}
    	if(stateName.equals("cooked")){
			isCooked = (stateVal.equals("true"));
			if(isCooked){
				temp = 60;
				color = cookedColor;
			} else {
				temp = 0;
				color = rawColor;
			}
		}
		super.setState(stateName, stateVal);
    }
}
