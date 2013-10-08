package probcog.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.Map;

import april.jmat.LinAlg;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;

public class SimSteak extends SimBoxPC{
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
        this.isCooked = ins.readString().equals("true");
        this.addNewState("cooked", new String[]{"false", "true"});
        
        if(isCooked){
        	// Dark Brown
        	this.color = new Color(0.4f, 0.2f, 0.1f, 1.0f);
        	this.setState("cooked", "true");
        } else {
        	// Pink
        	this.color = new Color(1.0f, .7f, .6f, 1.0f);
        }
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
        outs.writeDoubles(LinAlg.matrixToXyzrpy(T));
        outs.writeDouble(scale);
        outs.writeString(isCooked ? "true" : "false");
    }
    
    public void setState(String stateName, String stateVal){
    	if(stateName.equals("cooking") && stateVal.equals("true")){
    		if(!isCooked){
    			temp++;
    			if(temp > 60){
    				isCooked = true;
    	        	this.color = new Color(0.4f, 0.2f, 0.1f, 1.0f);
    	        	super.setState("cooked", "true");
    			}
    		}
    	}
    }
}
