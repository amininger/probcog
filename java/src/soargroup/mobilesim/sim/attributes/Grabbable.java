package soargroup.mobilesim.sim.attributes;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;
import soargroup.mobilesim.util.ResultTypes.*;

public class Grabbable implements IAttribute {
	private boolean _isGrabbed = false;
	public Grabbable(){
	}

	public boolean isGrabbed(){
		return _isGrabbed;
	}

	public void setGrabbed(boolean isGrabbed){
		this._isGrabbed = isGrabbed;
	}

	// Rules for Handling PickUp Action
	private static class PickUpRules implements ValidateRule<PickUpAction>, ApplyRule<PickUpAction> {
		// PickUpAction Valid: If Grabbable -> valid if not grabbed
		public IsValid validate(PickUpAction pickup){
			Grabbable grabbable = pickup.object.getAttr(Grabbable.class);
			if(grabbable == null || !grabbable.isGrabbed()){
				return IsValid.True();
			}
			return IsValid.False("Grabbable: The object is already grabbed");
		}
		// PickUpAction Apply: Update isGrabbed flag on object
		public Result apply(PickUpAction pickup){
			Grabbable grabbable = pickup.object.getAttr(Grabbable.class);
			if(grabbable != null){
				grabbable.setGrabbed(true);
			}
			return Result.Ok();
		}
	}
	static {
		PickUpRules rules = new PickUpRules();
		ActionHandler.addValidateRule(PickUpAction.class, rules);
		ActionHandler.addApplyRule(PickUpAction.class, rules);
	}

	//public Shape getShape() {
	//	if(isHeld){
	//		// Don't have collision on, otherwise the robot won't drive
	//		return new april.sim.SphereShape(0.0);
	//	} else {
	//		return new BoxShape(scale_xyz);
	//	}
	//}

}
