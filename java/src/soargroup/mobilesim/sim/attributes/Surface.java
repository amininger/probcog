package soargroup.mobilesim.sim.attributes;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.sim.RosieSimObject;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;
import soargroup.mobilesim.util.ResultTypes.*;

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

	@Override
	protected void setupRules(){
		//  PlaceObject: Valid if Relation == ON
		ActionHandler.addValidateRule(PlaceObject.class, new ValidateRule<PlaceObject>() {
			public IsValid validate(PlaceObject place){
				if(place.relation.equals(RosieConstants.REL_ON)){
					return IsValid.True();
				}
				return IsValid.False("Surface: relation must be on");
			}
		});
	}
}
