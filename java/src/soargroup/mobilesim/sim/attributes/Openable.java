package soargroup.mobilesim.sim.attributes;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.util.ResultTypes.*;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;

public class Openable extends Attribute {
	private boolean _isOpen = false;

	public Openable(RosieSimObject baseObject, boolean isOpen){
		super(baseObject);
		this.setOpen(isOpen);
	}

	public boolean isOpen(){
		return _isOpen;
	}

	public void setOpen(boolean isOpen){
		this._isOpen = isOpen;
		baseObject.setProperty(RosieConstants.DOOR, 
				isOpen ? RosieConstants.DOOR_OPEN : RosieConstants.DOOR_CLOSED);
	}

	// Registering Action Handling Rules
	static {
		// SetProp.Open: Valid if the object is Openable
		ActionHandler.addValidateRule(SetProp.Open.class, new ValidateRule<SetProp.Open>() {
			public IsValid validate(SetProp.Open open){
				if(open.object.is(Openable.class)){
					return IsValid.True();
				}
				return IsValid.False("Openable: Object is not openable");
			}
		});
		// SetProp.Open Apply: Update isOpen flag on object
		ActionHandler.addApplyRule(SetProp.Open.class, new ApplyRule<SetProp.Open>() {
			public Result apply(SetProp.Open open){
				Openable openable = open.object.getAttr(Openable.class);
				if(openable == null){
					return Result.Err("Openable: Object is not openable");
				}
				openable.setOpen(true);
				open.object.recreateVisObject();
				return Result.Ok();
			}
		});

		// SetProp.Close: Valid if the object is Openable
		ActionHandler.addValidateRule(SetProp.Close.class, new ValidateRule<SetProp.Close>() {
			public IsValid validate(SetProp.Close close){
				if(close.object.is(Openable.class)){
					return IsValid.True();
				}
				return IsValid.False("Openable: Object is not openable");
			}
		});
		// SetProp.Close Apply: Update isOpen flag on object
		ActionHandler.addApplyRule(SetProp.Close.class, new ApplyRule<SetProp.Close>() {
			public Result apply(SetProp.Close close){
				Openable openable = close.object.getAttr(Openable.class);
				if(openable == null){
					return Result.Err("Openable: Object is not openable");
				}
				openable.setOpen(false);
				close.object.recreateVisObject();
				return Result.Ok();
			}
		});
	}
}
