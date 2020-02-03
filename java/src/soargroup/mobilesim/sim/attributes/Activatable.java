package soargroup.mobilesim.sim.attributes;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;
import soargroup.mobilesim.util.ResultTypes.*;

public class Activatable extends Attribute {
	private boolean _isOn = false;

	public Activatable(RosieSimObject baseObject){
		super(baseObject);
	}

	public boolean isOn(){
		return _isOn;
	}

	public void setOn(boolean isOn){
		this._isOn = isOn;
	}
	
	// Registering Action Handling Rules
	static {
		// SetProp.TurnOn: Valid if the object is Activatable
		ActionHandler.addValidateRule(SetProp.TurnOn.class, new ValidateRule<SetProp.TurnOn>() {
			public IsValid validate(SetProp.TurnOn turnon){
				if(turnon.object.is(Activatable.class)){
					return IsValid.True();
				}
				return IsValid.False("Activatable: Object is not activatable");
			}
		});
		// SetProp.TurnOn Apply: Update isOn flag on object
		ActionHandler.addApplyRule(SetProp.TurnOn.class, new ApplyRule<SetProp.TurnOn>() {
			public Result apply(SetProp.TurnOn turnon){
				Activatable activatable = turnon.object.getAttr(Activatable.class);
				if(activatable == null){
					return Result.Err("Activatable: Object is not activatable");
				}
				activatable.setOn(true);
				return Result.Ok();
			}
		});

		// SetProp.TurnOff: Valid if the object is Activatable
		ActionHandler.addValidateRule(SetProp.TurnOff.class, new ValidateRule<SetProp.TurnOff>() {
			public IsValid validate(SetProp.TurnOff turnoff){
				if(turnoff.object.is(Activatable.class)){
					return IsValid.True();
				}
				return IsValid.False("Activatable: Object is not activatable");
			}
		});
		// SetProp.TurnOff Apply: Update isOn flag on object
		ActionHandler.addApplyRule(SetProp.TurnOff.class, new ApplyRule<SetProp.TurnOff>() {
			public Result apply(SetProp.TurnOff turnoff){
				Activatable activatable = turnoff.object.getAttr(Activatable.class);
				if(activatable == null){
					return Result.Err("Activatable: Object is not activatable");
				}
				activatable.setOn(true);
				return Result.Ok();
			}
		});
	}
}
