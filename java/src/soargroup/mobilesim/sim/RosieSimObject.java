package soargroup.mobilesim.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import soargroup.mobilesim.util.BoundingBox;
import april.jmat.LinAlg;
import april.util.*;
import april.sim.*;
import april.vis.*;

import soargroup.mobilesim.lcmtypes.object_data_t;
import soargroup.mobilesim.lcmtypes.classification_t;

public abstract class RosieSimObject implements SimObject{
	// Pose is the center of the object's bounding box
	protected double[] xyzrpy = new double[6];
	// Scale is in relation to a unit cube centered at the xyz coordinate
	protected double[] scale_xyz = new double[]{ 1, 1, 1 };

	protected Integer id;
    protected String desc;
	protected HashMap<String, String> properties = new HashMap<String, String>();

	private static int NEXT_ID = 1;

    public RosieSimObject(SimWorld sw){
		id = RosieSimObject.NEXT_ID;
		RosieSimObject.NEXT_ID += 1;
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

    public synchronized void setRunning(boolean isRunning){ }

	public abstract void performDynamics(ArrayList<RosieSimObject> worldObjects);
    
    public abstract VisObject getVisObject();

    public abstract Shape getShape();
    
    public BoundingBox getBoundingBox(){
    	return new BoundingBox(scale_xyz, xyzrpy);
    }

	public object_data_t getObjectData(){
		object_data_t objdat = new object_data_t();
		objdat.utime = TimeUtil.utime();
		objdat.id = id;

		objdat.xyzrpy = LinAlg.copy(xyzrpy);
		objdat.lenxyz = LinAlg.copy(scale_xyz);

		objdat.num_classifications = 0;
		objdat.classifications = new classification_t[properties.size()];
		for(Map.Entry<String, String> e : properties.entrySet()){
			classification_t cls = new classification_t();
			cls.category = e.getKey();
			cls.name = e.getValue();
			cls.confidence = 1.0;
			objdat.classifications[objdat.num_classifications] = cls;
			objdat.num_classifications += 1;
		}
		return objdat;
	}

    
    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
        // [Str] description
        desc = ins.readString();

    	// [Dbl]x3 xyz center of object
    	double[] xyz = ins.readDoubles();

		// [Dbl] rotation (xy plane)
		double yaw = ins.readDouble();
		xyzrpy = new double[]{ xyz[0], xyz[1], xyz[2], 0.0, 0.0, yaw };

    	// [Dbl]x3 scale xyz
    	scale_xyz = ins.readDoubles();

		// [Int] number of properties
		int num_props = ins.readInt();
		// [Str]x2 for each property (property, value)
		for(int i = 0; i < num_props; i += 1){
			String[] prop = ins.readString().trim().split("=");
			properties.put(prop[0], prop[1]);
		}
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeDoubles(LinAlg.copy(xyzrpy, 3));
		outs.writeDouble(xyzrpy[5]);
    	outs.writeDoubles(scale_xyz);
		outs.writeInt(properties.size());
		for(Map.Entry<String, String> e : properties.entrySet()){
			outs.writeString(e.getKey());
			outs.writeString(e.getValue());
		}
    }
}
