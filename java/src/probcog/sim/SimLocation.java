package probcog.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import probcog.classify_old.Classifications;
import probcog.classify_old.Features;
import probcog.classify_old.Features.FeatureCategory;
import probcog.perception.Obj;
import probcog.sim_old.SimObjectPC;
import probcog.util.BoundingBox;
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

public class SimLocation extends SimObjectPC implements SimObject
{
    protected double T[][] = LinAlg.identity(4);  // position
	protected static Color  color = Color.gray;
	
	protected double[] minPoint = new double[]{ -1.0, -1.0 };
	protected double[] maxPoint = new double[]{  1.0,  1.0 };
	
	protected String handle = null;

    public SimLocation(SimWorld sw)
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
    
    public boolean posIsInside(double[] xyzrpy){
    	return xyzrpy[0] >= minPoint[0] && xyzrpy[1] >= minPoint[1] &&
    	       xyzrpy[0] <= maxPoint[0] && xyzrpy[1] <= maxPoint[1];
    }
    
    public VisObject getVisObject()
    {
        ArrayList<Object> objs = new ArrayList<Object>();

        double len = (maxPoint[0] - minPoint[0]);
        double wid = (maxPoint[1] - minPoint[1]);
        
        // Actual Rectangle
        objs.add(new VisChain(new VzRectangle(len, wid, new VzMesh.Style(color))));

        // The handle of the location
        objs.add(new VisChain(LinAlg.translate(0,0,0),
                              LinAlg.scale(0.05),
                              new VzText(VzText.ANCHOR.CENTER, String.format("<<black>> %s", handle))));

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
        double c1[] = ins.readDoubles();
    	
    	// 2nd corner [x, y]
        double c2[] = ins.readDoubles();
        
        minPoint[0] = Math.min(c1[0], c2[0]);
        minPoint[1] = Math.min(c1[1], c2[1]);
        maxPoint[0] = Math.max(c1[0], c2[0]);
        maxPoint[1] = Math.max(c1[1], c2[1]);
        
        double x = (minPoint[0] + maxPoint[0])/2;
        double y = (minPoint[1] + maxPoint[1])/2;
        double[] xyzrpy = new double[]{x, y, 0.001, 0.0, 0.0, 0.0};
        this.T = LinAlg.xyzrpyToMatrix(xyzrpy);
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeString(handle);
    	outs.writeDoubles(minPoint);
    	outs.writeDoubles(maxPoint);
    }

    public void setRunning(boolean b){
    	
    }
}
