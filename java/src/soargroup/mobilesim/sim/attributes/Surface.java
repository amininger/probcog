package soargroup.mobilesim.sim.attributes;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.util.ResultTypes.*;
import soargroup.mobilesim.sim.RosieSimObject;
import soargroup.mobilesim.sim.actions.PutDown;
import soargroup.mobilesim.sim.actions.ActionHandler;

public class Surface extends ObjectHolder {
	public static final double ANCHOR_SPACING = 0.40;

	// Setup anchors with default spacing
	public Surface(RosieSimObject object, boolean useDefaultSpacing){
		super(object);
		if(useDefaultSpacing){
			// Default uses whole width/height and puts anchors at the top
			double[] scale = object.getScale();
			addPoints(scale[0], scale[1], scale[2]/2+0.001, ANCHOR_SPACING);
		}
	}

	static {
		//  PutDown.Target: NotValid if target is a Surface and relation != ON
		ActionHandler.addValidateRule(PutDown.Target.class, new ActionHandler.ValidateRule<PutDown.Target>() {
			public IsValid validate(PutDown.Target putdown){
				if(putdown.target.is(Surface.class) && !putdown.relation.equals(RosieConstants.REL_ON)){
					return IsValid.False("Surface: relation must be on");
				}
				return IsValid.True();
			}
		});
	}
}
