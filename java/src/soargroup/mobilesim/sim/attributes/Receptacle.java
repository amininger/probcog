package soargroup.mobilesim.sim.attributes;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.util.ResultTypes.*;
import soargroup.mobilesim.sim.RosieSimObject;
import soargroup.mobilesim.sim.actions.PutDown;
import soargroup.mobilesim.sim.actions.ActionHandler;

public class Receptacle extends ObjectHolder {
	public static final double ANCHOR_SPACING = 0.40;

	// Setup anchors with default spacing
	public Receptacle(RosieSimObject object, boolean useDefaultSpacing){
		super(object);
		if(useDefaultSpacing){
			// Default uses whole width/height and puts anchors at the bottom
			double[] scale = object.getScale();
			addPoints(scale[0], scale[1], -scale[2]/2+0.001, ANCHOR_SPACING);
		}
	}

	// Receptacle with custom custom surface boundaries and point spacing
	public Receptacle(RosieSimObject object, double dx, double dy, double z, double spacing){
		super(object);
		addPoints(dx, dy, z, spacing);
	}

	static {
		//  PutDown.Target: NotValid if target is a Receptacle and relation != IN
		ActionHandler.addValidateRule(PutDown.Target.class, new ActionHandler.ValidateRule<PutDown.Target>() {
			public IsValid validate(PutDown.Target putdown){
				if(putdown.target.is(Receptacle.class) && !putdown.relation.equals(RosieConstants.REL_IN)){
					return IsValid.False("Receptacle: relation must be in");
				}
				return IsValid.True();
			}
		});
	}
}
