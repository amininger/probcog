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
import soargroup.mobilesim.vis.VzOpenBox;
import soargroup.mobilesim.util.ResultTypes.*;
import soargroup.mobilesim.sim.attributes.*;
import soargroup.mobilesim.sim.actions.*;

// Lcm Types
import soargroup.mobilesim.lcmtypes.object_data_t;
import soargroup.mobilesim.lcmtypes.classification_t;

public class RosieSimObject extends BaseSimObject{
	protected enum ModelType {
		BOX, OPENBOX, CUSTOM
	}
	protected Integer id;
	protected String desc;
	protected Color  color = Color.gray;
	protected ModelType model = ModelType.BOX;
	protected HashMap<String, String> properties = new HashMap<String, String>();

	private static int NEXT_ID = 1;

	private boolean _isVisible = true;

	public RosieSimObject(SimWorld sw){
		super(sw);
		id = RosieSimObject.NEXT_ID;
		RosieSimObject.NEXT_ID += 1;
	}

	// A RosieSimObject is composed of Attributes that define its behavior 
	//    and how it responds to different actions
	// (e.g., if it is Grabbable, then you can issue PickUp commands for the object)
	protected HashMap<Class<?>, Attribute> attributes = new HashMap<Class<?>, Attribute>();
	private HashSet<Attribute> uniqueAttrs = new HashSet<Attribute>();
	public <T extends Attribute> boolean is(Class<T> cls){
		return attributes.containsKey(cls);
	}
	public <T extends Attribute> T getAttr(Class<T> cls){
		Attribute attr = attributes.get(cls);
		return attr == null ? null : (T)attr;
	}

	protected void addAttribute(Attribute attr){
		uniqueAttrs.add(attr);
		// Add any supertype Attributes as well (e.g. adding Surface also adds ObjectHolder)
		Class cls = attr.getClass();
		while(Attribute.class.isAssignableFrom(cls) && cls != Attribute.class){
			attributes.put(cls, attr);
			cls = cls.getSuperclass();
		}

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

	public boolean isVisible(){
		return _isVisible;
	}

	@Override
	public void setXYZRPY(double[] newpose){
		for(Attribute attr : uniqueAttrs){
			attr.moveHandler(newpose);
		}
		this.xyzrpy = newpose;
	}

	public void init(ArrayList<SimObject> worldObjects) { 
		addAttribute(new InRegion(this));
		setupRules();
	}

	protected void setupRules(){ }

	// Action Handling Rules
	static {
		// PickUp Apply: Make object non-collidable
		ActionHandler.addApplyRule(PickUp.class, new ActionHandler.ApplyRule<PickUp>() {
			public Result apply(PickUp pickup){
				pickup.object.collide = false;
				return Result.Ok();
			}
		});
		// PutDown Apply: Make object collidable again
		ActionHandler.addApplyRule(PutDown.class, new ActionHandler.ApplyRule<PutDown>() {
			public Result apply(PutDown putdown){
				putdown.object.collide = true;
				return Result.Ok();
			}
		});
		//// SetProp Apply: Change the property
		//ActionHandler.addApplyRule(SetProp.class, new ActionHandler.ApplyRule<SetProp>() {
		//	public Result apply(SetProp setprop){
		//		setprop.object.setProprety(setprop.property, setprop.value);
		//		return Result.Ok();
		//	}
		//});
	}

	// Children can override to implement any dynamics, this is called multiple times/second
	// dt is time elapsed since last update (fraction of a second)
	public void update(double dt) { 
		for(Attribute attr : uniqueAttrs){
			attr.update(dt);
		}
	}

	public void setProperty(String property, String value){
		properties.put(property, value);
	}

	@Override
	public VisObject getVisObject(){
		if(visObject == null){
			VisChain vc = createVisObject();
			for(Attribute attr : uniqueAttrs){
				attr.render(vc);
			}
			visObject = vc;
		}
		return visObject;
	}

	@Override
	public VisChain createVisObject() {
		VisChain vc = new VisChain();
		switch(model){
			case BOX:
				// This is a wireframe box
				//	return new VzBox(scale_xyz, new VzLines.Style(color, 1));
				vc.add(new VzBox(scale_xyz, new VzMesh.Style(color)));
				break;
			case OPENBOX:
				vc.add(new VzOpenBox(scale_xyz, new VzMesh.Style(color)));
				break;
			case CUSTOM:
				String modelName = properties.get("category").replaceAll("\\d", "");
				VisChain vobj = ObjectModels.createModel(modelName, scale_xyz, color);
				if(vobj != null){
					vc.add(vobj);
				}
		}
		return vc;
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

		super.read(ins); // xyz rot scale_xyz

		// [Int]x3 rgb color
		int rgb[] = ins.readInts();
		color = new Color(rgb[0], rgb[1], rgb[2]);

		// [Int] number of properties
		int num_props = ins.readInt();
		// [Str]x2 for each property (property, value)
		for(int i = 0; i < num_props; i += 1){
			String prop = ins.readString().toLowerCase().trim();
			if(prop.equals("grabbable")){
				addAttribute(new Grabbable(this));
			} else if(prop.equals("openbox")){
				model = ModelType.OPENBOX;
			} else if(prop.equals("model")){
				model = ModelType.CUSTOM;
			} else if(prop.contains("=")){
				String[] splitProp = prop.split("=");
				properties.put(splitProp[0], splitProp[1]);
			}
		}
	}

	/** Write one or more lines that serialize this instance. No line
	 * is allowed to consist of just an asterisk. **/
	public void write(StructureWriter outs) throws IOException
	{
		outs.writeString(desc);
		super.write(outs);

		int rgb[] = new int[]{ color.getRed(), color.getGreen(), color.getBlue() };
    	outs.writeInts(rgb);

		outs.writeInt(properties.size());
		for(Map.Entry<String, String> e : properties.entrySet()){
			outs.writeString(e.getKey());
			outs.writeString(e.getValue());
		}
	}
}
