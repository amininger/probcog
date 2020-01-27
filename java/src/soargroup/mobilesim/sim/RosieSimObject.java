package soargroup.mobilesim.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.*;

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

	// Anchors are locations where objects can be placed
	protected ArrayList<AnchorPoint> anchors = new ArrayList<AnchorPoint>();

	protected SimRegion curRegion = null;
	protected boolean staleRegion = true;

	private VisObject visObject = null;
	private boolean staleVis = true;

	private boolean isHeld = false;

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

	public void setXYZRPY(double[] newpose){
		for(AnchorPoint pt : anchors){ pt.checkObject(); }
		this.staleRegion = true;
		this.xyzrpy = newpose;
		for(AnchorPoint pt : anchors){ pt.move(); }
	}

	public double[][] getPose() {
		return LinAlg.xyzrpyToMatrix(xyzrpy);
	}

	public void setPose(double T[][]) {
		this.setXYZRPY(LinAlg.matrixToXyzrpy(T));
	}	

	public double[] getScale(){
		return LinAlg.copy(scale_xyz);
	}

	public void setIsHeld(boolean isHeld){
		this.isHeld = isHeld;
	}

	public synchronized void setRunning(boolean isRunning){ }

	public abstract void performDynamics(ArrayList<SimObject> worldObjects);

	public abstract VisChain createVisObject();

	// Return the closest region from the list that contains the position of the object
	//   It will cache this result in curRegion and not recompute until its position changes
	public SimRegion getRegion(List<SimRegion> regions){
		if(staleRegion){
			Double bestDist = Double.MAX_VALUE;
			curRegion = null;
			for(SimRegion region : regions){
				if(region.contains(xyzrpy)){
					double dist2 = region.getDistanceSq(xyzrpy);
					if(dist2 < bestDist){
						bestDist = dist2;
						curRegion = region;
					}
				}
			}
			staleRegion = false;
		}
		return curRegion;
	}

	public void setState(String property, String value){
		if(properties.containsKey(property)){
			properties.put(property, value);
		} else {
			System.err.println("Object " + desc + " does not recognize property " + property);
		}
	}

	public boolean addObject(RosieSimObject obj, String relation){
		for(AnchorPoint pt : anchors){
			if(pt.relation.equals(relation) && !pt.hasObject()){
				pt.addObject(obj);
				return true;
			}
		}
		System.err.println("Error adding " + obj.getDescription() + " to " + 
				desc + ", anchors are full");
		return false;
	}

	protected void recreateVisObject(){
		this.staleVis = true;
	}

	public VisObject getVisObject(){
		if(this.staleVis){
			VisChain vc = createVisObject();
			// Uncomment to also draw anchor points
			for(AnchorPoint anchor : anchors){
				vc.add(new VisChain(LinAlg.translate(anchor.xyz), 
							new VzBox(new double[]{ 0.04, 0.04, 0.04}, new VzMesh.Style(Color.black))));
			}
			visObject = vc;
			this.staleVis = false;
		}
		return visObject;
	}

	public Shape getShape() {
		if(isHeld){
			// Don't have collision on, otherwise the robot won't drive
			return new april.sim.SphereShape(0.0);
		} else {
			return new BoxShape(scale_xyz);
		}
	}
	
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
		outs.writeString(desc);
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
