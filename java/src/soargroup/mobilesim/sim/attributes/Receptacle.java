package soargroup.mobilesim.sim.attributes;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.util.ResultTypes.*;
import soargroup.mobilesim.sim.RosieSimObject;
import soargroup.mobilesim.sim.actions.PlaceObject;
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

	@Override
	protected void setupRules(){
		//  PlaceObject: Valid if relation == IN
		ActionHandler.addValidateRule(PlaceObject.class, new ActionHandler.ValidateRule<PlaceObject>() {
			public IsValid validate(PlaceObject place){
				if(place.relation.equals(RosieConstants.REL_IN)){
					return IsValid.True();
				}
				return IsValid.False("Receptacle: relation must be in");
			}
		});
	}
}
