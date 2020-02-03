package soargroup.mobilesim.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.*;

import soargroup.mobilesim.util.BoundingBox;
import april.jmat.LinAlg;
import april.util.*;
import april.sim.*;
import april.vis.*;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.lcmtypes.object_data_t;
import soargroup.mobilesim.lcmtypes.classification_t;
import soargroup.mobilesim.sim.attributes.*;

public abstract class RosieSimObject extends BaseSimObject{
	protected Integer id;
	protected String desc;
	protected HashMap<String, String> properties = new HashMap<String, String>();

	protected SimRegion curRegion = null;
	protected boolean staleRegion = true;

	private static int NEXT_ID = 1;

	protected Double temperature = 70.0;

	public RosieSimObject(SimWorld sw){
		super(sw);
		id = RosieSimObject.NEXT_ID;
		RosieSimObject.NEXT_ID += 1;
		properties.put(RosieConstants.TEMPERATURE, temperature.toString());
	}

	// A RosieSimObject is composed of Attributes that define its behavior 
	//    and how it responds to different actions
	// (e.g., if it is Grabbable, then you can issue PickUp commands for the object)
	protected HashMap<Class<?>, Attribute> attributes = new HashMap<Class<?>, Attribute>();
	public <T extends Attribute> boolean is(Class<T> cls){
		return attributes.containsKey(cls);
	}
	public <T extends Attribute> T getAttr(Class<T> cls){
		Attribute attr = attributes.get(cls);
		return attr == null ? null : (T)attr;
	}

	
	public Integer getID(){
		return id;
	}

	public String getDesc(){
		return desc;
	}

	public String toString(){
		return desc + "_" + id.toString();
	}

	@Override
	public void setXYZRPY(double[] newpose){
		this.staleRegion = true;
		this.xyzrpy = newpose;
		for(Attribute attr : attributes.values()){
			attr.moveHandler(newpose);
		}
	}

	// Doesn't directly set temperature, instead will gradually move towards the given value
	public void changeTemperature(double targetTemp){
		temperature += (targetTemp - temperature) * 0.02; 
		properties.put(RosieConstants.TEMPERATURE, temperature.toString());
	}


	// Children can override to do any initialization once all world objects are created
	public void setup(ArrayList<SimObject> worldObjects) { 
		attributes.put(Grabbable.class, new Grabbable(this));
		setupRules();
	}

	private void setupRules(){}

	// Children can override to implement any dynamics, this is called multiple times/second
	public void performDynamics(ArrayList<SimObject> worldObjects) { }

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

	@Override
	public VisObject getVisObject(){
		if(visObject == null){
			VisChain vc = createVisObject();
			for(Attribute attr : attributes.values()){
				attr.render(vc);
			}
			visObject = vc;
		}
		return visObject;
	}

	//Receptacle
//	@Override
//	public VisChain createVisObject() {
//		// Use an OpenBox instead of a regular box
//		VisChain c = new VisChain();
//		c.add(new VzOpenBox(scale_xyz, new VzMesh.Style(color)));
//		return c;
//	}

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

		super.read(ins); // xyzryp scale_xyz

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
		super.write(outs);
		outs.writeInt(properties.size());
		for(Map.Entry<String, String> e : properties.entrySet()){
			outs.writeString(e.getKey());
			outs.writeString(e.getValue());
		}
	}
}
