package soargroup.mobilesim.sim.actions;

import soargroup.mobilesim.sim.*;

public class PickUpAction extends Action {
	public final RosieSimObject object;
	public PickUpAction(RosieSimObject object){
		this.object = object;
	}

	public String toString(){
		return "PickUp(" + object.toString() + ")";
	}

//	public abstract class ValidRule {
//		public abstract IsValid isValid(PickUpAction pickUp);
//	}
//	private static HashSet<ValidRule> validRules = new HashSet<ValidRule>();
//	public static void addValidRule(ValidRule validRule){
//		validRules.add(validRule);
//	}
//
//	public abstract class ApplyRule {
//		public abstract Result apply(PickUpAction pickUp);
//	}
//	private static HashSet<ApplyRule> applyRules = new HashSet<ApplyRule>();
//	public static void addApplyRule(ApplyRule applyRule){
//		applyRules.add(applyRule);
//	}

}
