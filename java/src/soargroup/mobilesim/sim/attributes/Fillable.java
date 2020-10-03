package soargroup.mobilesim.sim.attributes;

import java.util.ArrayList;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.util.ResultTypes.*;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;

public class Fillable extends Attribute {
	private boolean _isFull = false;

	public Fillable(RosieSimObject baseObject, boolean isFull){
		super(baseObject);
		this.setFull(isFull);
	}

	public boolean isFull(){
		return _isFull;
	}

	public void setFull(boolean isFull){
		this._isFull = isFull;
		baseObject.setProperty(RosieConstants.FILLED, 
				isFull ? RosieConstants.FULL : RosieConstants.EMPTY);
	}

//	// Registering Action Handling Rules
//	static {
//		// SetProp.Full: Valid if the object is Fillable and not open
//		ActionHandler.addValidateRule(SetProp.Full.class, new ValidateRule<SetProp.Full>() {
//			public IsValid validate(SetProp.Full open){
//				Fillable openable = open.object.getAttr(Fillable.class);
//				if(openable == null){
//					return IsValid.False("Fillable: Object is not openable");
//				}
//				if(openable.isFull()){
//					return IsValid.False("Fillable: Object is already open");
//				}
//				return IsValid.True();
//			}
//		});
//		// SetProp.Full Apply: Update isFull flag on object
//		ActionHandler.addApplyRule(SetProp.Full.class, new ApplyRule<SetProp.Full>() {
//			public Result apply(SetProp.Full open){
//				Fillable openable = open.object.getAttr(Fillable.class);
//				if(openable == null){
//					return Result.Err("Fillable: Object is not openable");
//				}
//				openable.setFull(true);
//				open.object.recreateVisObject();
//				return Result.Ok();
//			}
//		});
//
//		// SetProp.Close: Valid if the object is Fillable and open
//		ActionHandler.addValidateRule(SetProp.Close.class, new ValidateRule<SetProp.Close>() {
//			public IsValid validate(SetProp.Close close){
//				Fillable openable = close.object.getAttr(Fillable.class);
//				if(openable == null){
//					return IsValid.False("Fillable: Object is not openable");
//				}
//				if(!openable.isFull()){
//					return IsValid.False("Fillable: Object is not open");
//				}
//				return IsValid.True();
//			}
//		});
//		// SetProp.Close Apply: Update isFull flag on object
//		ActionHandler.addApplyRule(SetProp.Close.class, new ApplyRule<SetProp.Close>() {
//			public Result apply(SetProp.Close close){
//				Fillable openable = close.object.getAttr(Fillable.class);
//				if(openable == null){
//					return Result.Err("Fillable: Object is not openable");
//				}
//				openable.setFull(false);
//				close.object.recreateVisObject();
//				return Result.Ok();
//			}
//		});
//		// PutDown.Target Apply: If the target is closed, make the object not visible 
//		// (This should only happen due to simulator teleporting objects for testing, etc.)
//		ActionHandler.addApplyRule(PutDown.Target.class, new ApplyRule<PutDown.Target>() {
//			public Result apply(PutDown.Target putdown){
//				Fillable openable = putdown.target.getAttr(Fillable.class);
//				if(openable != null && !openable.isFull()){
//					putdown.object.setVisible(false);
//				}
//				return Result.Ok();
//			}
//		});
//	}
}
