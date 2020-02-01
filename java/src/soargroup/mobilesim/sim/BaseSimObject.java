package soargroup.mobilesim.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import soargroup.mobilesim.util.BoundingBox;
import april.jmat.LinAlg;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.sim.*;
import april.vis.*;

public abstract class BaseSimObject implements SimObject{
	// Pose is the center of the object's bounding box
	protected double xyzrpy[] = new double[6];
	// Scale is in relation to a unit cube centered at the xyz coordinate
	protected double scale_xyz[] = new double[]{ 1, 1, 1 };

	private VisObject visObject = null;

	protected boolean isRunning = false;
	protected boolean collide = true;

	public BaseSimObject(SimWorld sw){
	}
	
	// XYZRPY
	public double[] getXYZRPY(){
		return xyzrpy;
	}

	public void setXYZRPY(double[] newpose){
		this.xyzrpy = newpose;
	}

	// Pose double[][]
	public double[][] getPose(){
		return LinAlg.xyzrpyToMatrix(xyzrpy);
	}

	public void setPose(double T[][]){
		this.setXYZRPY(LinAlg.matrixToXyzrpy(T));
	}	

	// Scale
	public double[] getScale(){
		return LinAlg.copy(scale_xyz);
	}

	// isRunning
	public void setRunning(boolean isRunning){ 
		this.isRunning = isRunning;
	}

	// VisObject
	//   createVisObject - Children must implement this,
	//   create a VisChain that will model the object
	//   this result is cached, call recreateVisObject() to invalidate the cached VisObject
	public abstract VisChain createVisObject();

	protected void recreateVisObject(){  this.visObject = null;  }

	public VisObject getVisObject(){
		if(visObject == null){
			visObject = createVisObject();
		}
		return visObject;
	}

	// Shape
	public Shape getShape(){
		if(collide){
			// Use the bounding box 
			return new BoxShape(scale_xyz);
		} else {
			// Don't have collision on, otherwise the robot won't drive
			return new april.sim.SphereShape(0.0);
		}
	}
	
	public BoundingBox getBoundingBox(){
		return new BoundingBox(scale_xyz, xyzrpy);
	}
	
	/** Restore state that was previously written **/
	public void read(StructureReader ins) throws IOException
	{
		// [Dbl]x3 xyz center of object
		double[] xyz = ins.readDoubles();

		// [Dbl] rotation (xy plane)
		double yaw = ins.readDouble();
		xyzrpy = new double[]{ xyz[0], xyz[1], xyz[2], 0.0, 0.0, yaw };

		// [Dbl]x3 scale xyz
		scale_xyz = ins.readDoubles();
	}

	/** Write one or more lines that serialize this instance. No line
	 * is allowed to consist of just an asterisk. **/
	public void write(StructureWriter outs) throws IOException
	{
		outs.writeDoubles(LinAlg.copy(xyzrpy, 3));
		outs.writeDouble(xyzrpy[5]);
		outs.writeDoubles(scale_xyz);
	}
}
