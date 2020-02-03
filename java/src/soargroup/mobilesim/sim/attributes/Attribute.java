package soargroup.mobilesim.sim.attributes;

import april.vis.*;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;
import soargroup.mobilesim.util.ResultTypes.*;

public class Attribute {
	protected final RosieSimObject object;
	public Attribute(RosieSimObject object){
		this.object = object;
		setupRules();
	}

	// Use to set up any action handler rules
	protected void setupRules(){ }

	// Called when the object moves
	public void moveHandler(double[] xyzrpy){ }

	// Called when the object needs to recalculate its VisObject
	//   you can add elements to the VisChain, 
	//   any transforms are relative to the object itself (pose transformations already applied)
	public void render(VisChain vc){ }

}
