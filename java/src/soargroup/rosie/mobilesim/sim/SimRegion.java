package soargroup.rosie.mobilesim.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import soargroup.rosie.mobilesim.util.BoundingBox;
import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzMesh;
import april.vis.VzRectangle;
import april.vis.VzText;
import april.vis.VzLines;

public class SimRegion extends SimObjectPC implements SimObject
{
    protected double T[][] = LinAlg.identity(4);  // position
	protected static Color  color = new Color(175, 175, 175);
	
	protected double width = 1.0;
	protected double length = 1.0;
	
	protected String handle = null;

    public SimRegion(SimWorld sw)
    {
    	super(sw);
    }
    
    public String getHandle(){
    	return handle;
    }
    public void setHandle(String handle){
    	this.handle = handle;
    }

    public double[][] getPose()
    {
        return LinAlg.copy(T);
    }

    public void setPose(double T[][])
    {
        this.T = LinAlg.copy(T);
    }    
    
    public VisObject getVisObject()
    {
        ArrayList<Object> objs = new ArrayList<Object>();

        // Actual Rectangle
        //objs.add(new VisChain(new VzRectangle(width, length, new VzMesh.Style(color)), new VzRectangle(width+0.001, length+0.001, new VzLines.Style(Color.black, 2))));
        objs.add(new VzRectangle(width, length, new VzMesh.Style(color)));

        // The handle of the location
       // objs.add(new VisChain(LinAlg.translate(0,0,0.01),
       //                       LinAlg.scale(0.05),
       //                       new VzText(VzText.ANCHOR.CENTER, String.format("<<black>> %s", handle))));

        return new VisChain(objs.toArray());
    }

    public Shape getShape()
    {
    	return new april.sim.SphereShape(0.0);
    }

    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
    	// handle for the location
    	handle = ins.readString();
    	
    	// 1st corner [x, y]
    	double[] xyzrpy = ins.readDoubles();
    	
    	width = ins.readDouble();
    	length = ins.readDouble();

        this.T = LinAlg.xyzrpyToMatrix(xyzrpy);
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeString(handle);
    	outs.writeDoubles(LinAlg.matrixToXyzrpy(T));
    	outs.writeDouble(width);
    	outs.writeDouble(length);
    }

    public void setRunning(boolean b){
    	
    }
}
