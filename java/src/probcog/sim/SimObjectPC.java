package probcog.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import probcog.util.BoundingBox;
import april.jmat.LinAlg;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.sim.*;
import april.vis.*;

public abstract class SimObjectPC implements SimObject{
	protected double xyzrpy[] = new double[6];
	protected double scale_xyz[] = new double[]{ 1, 1, 1 };
	protected Color  color = Color.gray;
	protected HashMap<String, String> classifications;

    public SimObjectPC(SimWorld sw){
    	classifications = new HashMap<String, String>();
    }

    public double[][] getPose()
    {
    	return LinAlg.xyzrpyToMatrix(xyzrpy);
    }

    public void setPose(double T[][])
    {
    	this.xyzrpy = LinAlg.matrixToXyzrpy(T);
    }    
    
    public abstract VisObject getVisObject();

    public abstract Shape getShape();
    
    public BoundingBox getBoundingBox(){
    	return new BoundingBox(scale_xyz, xyzrpy);
    }
    
    public HashMap<String, String> getClassifications(){
    	return classifications;
    }

    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
    	// xyzrpy
    	xyzrpy = ins.readDoubles();
    	
    	// scale xyz
    	scale_xyz = ins.readDoubles();
    	
    	// rgb color
    	int rgb[] = ins.readInts();
    	color = new Color(rgb[0], rgb[1], rgb[2]);
    	
    	// classifications
    	int numClassifications = ins.readInt();
    	for (int i = 0; i < numClassifications; i++){
    		String[] words = ins.readString().trim().split("=");
    		if(words.length > 2){
    			continue;
    		}
    		String category = words[0];
    		String value = words[1];
    		classifications.put(category, value);
    	}
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeDoubles(xyzrpy);
    	outs.writeDoubles(scale_xyz);
    	int rgb[] = new int[]{ color.getRed(), color.getGreen(), color.getBlue() };
    	outs.writeInts(rgb);
    	outs.writeInt(classifications.size());
    	for(Map.Entry<String, String> e : classifications.entrySet()){
    		outs.writeString(e.getKey());
    		outs.writeString(e.getValue());
    	}
    }

    public void setRunning(boolean b){
    	
    }
}
