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
	protected Integer id;
    protected String desc;

    public SimObjectPC(SimWorld sw){
    }
    
    public Integer getID(){
    	return id;
    }

    public String getDescription(){
        return desc;
    }

    public double[] getXYZRPY(){
        return xyzrpy;
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
    
    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
    	// id of the object (used to generate a tag classification)
    	id = ins.readInt();

        // descriptio
        desc = ins.readString();
    	
    	// xyzrpy
    	xyzrpy = ins.readDoubles();
    	
    	// scale xyz
    	scale_xyz = ins.readDoubles();
    	
    	// rgb color
    	int rgb[] = ins.readInts();
    	color = new Color(rgb[0], rgb[1], rgb[2]);
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeInt(id);
    	outs.writeDoubles(xyzrpy);
    	outs.writeDoubles(scale_xyz);
    	int rgb[] = new int[]{ color.getRed(), color.getGreen(), color.getBlue() };
    	outs.writeInts(rgb);
    }

    public void setRunning(boolean b){
    	
    }
}
