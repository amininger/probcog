package probcog.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;

import april.jmat.LinAlg;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.sim.*;
import april.vis.*;

public class SimDoor implements SimObject {
	 protected static Color  color = Color.gray;
	 
	 protected static double width = 1.5;
	 protected static double thickness = 0.2;
	 protected static double height = 1.5;
	 
	 protected double T[][] = LinAlg.identity(4);  // position
	 protected double xyzrpy[] = new double[]{ 0.0, 0.0, height/2, 0.0, 0.0, 0.0 };
	 
	 protected boolean open = true;
	 protected String loc1 = null;
	 protected String loc2 = null;

	 public SimDoor(SimWorld sw)
	 {
		 
	 }

    public double[][] getPose()
    {
    	return LinAlg.xyzrpyToMatrix(xyzrpy);
    }

    public void setPose(double T[][])
    {
        this.xyzrpy = LinAlg.matrixToXyzrpy(T);
    }    
    
    public boolean isOpen(){
    	return open;
    }
    
    public VisObject getVisObject()
    {
        ArrayList<Object> objs = new ArrayList<Object>();
        
        if(!open){
        	return new VzBox(thickness, width, height, new VzMesh.Style(color));
        }

        return null;
    }

    public Shape getShape()
    {
    	if(!open){
    		return new BoxShape(thickness, width, height);
    	}
    	return new SphereShape(0);
    }

    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
    	xyzrpy = ins.readDoubles();
    	xyzrpy[2] = height/2;
    	
    	loc1 = ins.readString();
    	loc2 = ins.readString();
    	
    	open = (ins.readString().toLowerCase().equals("open"));
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeDoubles(xyzrpy);
    	outs.writeString(loc1);
    	outs.writeString(loc2);
    	outs.writeString(open ? "open" : "closed");
    }

    public void setRunning(boolean b){
            
    }
}
