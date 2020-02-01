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
	
	// Registering Action Handling Rules
	static {
		PickUpRules rules = new PickUpRules();
		ActionHandler.addApplyRule(PickUpAction.class, rules);
	}
	static {
		PutDownRules rules = new PutDownRules();
		ActionHandler.addApplyRule(PutDownAction.class, rules);
	}

	// Rules for Handling PickUp Action
	private static class PickUpRules implements ApplyRule<PickUpAction> {
		// PickUpAction Apply: Update isGrabbed flag on object
		public Result apply(PickUpAction pickup){
			Grabbable grabbable = pickup.object.getAttr(Grabbable.class);
			if(grabbable != null){
				grabbable.setGrabbed(true);
			}
			return Result.Ok();
		}
	}

	// Rules for Handling PutDown Action
	private static class PutDownRules implements ApplyRule<PutDownAction> {
		// PutDownAction Apply: Update isGrabbed flag on object
		public Result apply(PutDownAction putdown){
			Grabbable grabbable = putdown.object.getAttr(Grabbable.class);
			if(grabbable != null){
				grabbable.setGrabbed(false);
			}
			return Result.Ok();
		}
	}
}
