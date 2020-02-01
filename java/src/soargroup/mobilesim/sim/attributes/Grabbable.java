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
		// PickUp Apply: Update isGrabbed flag on object
		ActionHandler.addApplyRule(PickUp.class, new ApplyRule<PickUp>() {
			public Result apply(PickUp pickup){
				Grabbable grabbable = pickup.object.getAttr(Grabbable.class);
				if(grabbable != null){
					grabbable.setGrabbed(true);
				}
				return Result.Ok();
			}
		});

		// PutDown Apply: Update isGrabbed flag on object
		ActionHandler.addApplyRule(PutDown.class, new ApplyRule<PutDown>() {
			public Result apply(PutDown putdown){
				Grabbable grabbable = putdown.object.getAttr(Grabbable.class);
				if(grabbable != null){
					grabbable.setGrabbed(false);
				}
				return Result.Ok();
			}
		});
	}
}
