package soargroup.mobilesim.sim.actions;

import java.util.*;
import soargroup.mobilesim.util.ResultTypes.*;

public class ActionHandler {

	/************************ Handle Actions *************************/

	public static <A extends Action> Result handle(A action){
		if(action == null){
			return Result.Err("Could not parse action of type " + action.getClass());
		}
		IsValid isValid = ActionHandler.validate(action);
		if(isValid instanceof NotValid){
			NotValid notValid = (NotValid)isValid;
			return Result.Err("Action " + action.toString() + " is not valid\n" + 
					"Reason: " + notValid.reason);
		}
		Result result = ActionHandler.apply(action);
		if(result instanceof Err){
			Err err = (Err)result;
			return Result.Err("Action " + action.toString() + " failed\n" + 
					"Reason: " + err.reason);
		}
		return Result.Ok();
	}

	/*********************** Validate Actions *************************/
	// Before applying an action, we make sure it is valid
	
	public interface ValidateRule<A extends Action> {
		IsValid validate(A action);
	}

	// Any object can add a validation rule that must be checked before doing an action
	public static <A extends Action> void addValidateRule(Class<A> actionType, ValidateRule<A> rule){
		if(!validateRules.containsKey(actionType)){
			// If this is the first validate rule for a certain type, create a new list in the validateRules map
			validateRules.put(actionType, new ArrayList< ValidateRule<A> >());
		}
		List< ValidateRule<A> > rules = (List< ValidateRule<A> >)validateRules.get(actionType);
		rules.add(rule);
	}

	// Checks the given action against all the validation rules to make sure it is ok to apply
	public static <A extends Action> IsValid validate(A action) {
		// Loop through every ValidateRule for the given action type and make sure none of them are invalid
		IsValid isValid = IsValid.True();
		HashSet< ValidateRule<A> > rules = (HashSet< ValidateRule<A> >)validateRules.get(action.getClass());
		for(ValidateRule<A> rule : rules){
			isValid = isValid.AND(rule.validate(action));
		}
		return isValid;
	}

	// Map from an action class to a list of ValidateRules for that action type
	private static HashMap<Class<?>, Object>validateRules = new HashMap<Class<?>, Object>();


	/*********************** Apply Actions *************************/
	// once an action is validated, then apply it via the given handlers

	public interface ApplyRule<A extends Action> {
		Result apply(A action);
	}

	public static <A extends Action> void addApplyRule(Class<A> actionType, ApplyRule<A> rule){
		if(!applyRules.containsKey(actionType)){
			// If this is the first apply rule for a certain type, create a new list in the applyRules map
			applyRules.put(actionType, new ArrayList< ApplyRule<A> >());
		}
		List< ApplyRule<A> > rules = (List< ApplyRule<A> >)applyRules.get(actionType);
		rules.add(rule);
	}

	// Calls each apply rule involving the given action and reports whether it succeeded or failed
	public static <A extends Action> Result apply(A action) {
		// Loop through every ApplyRule for the given action type and make sure none of them fail
		List< ApplyRule<A> > rules = (List< ApplyRule<A> >)applyRules.get(action.getClass());
		for(ApplyRule<A> rule : rules){
			Result result = rule.apply(action);
			if(result instanceof Err){
				return result;
			}
		}
		return Result.Ok();
	}

	// Map from an action class to a list of ApplyRules for that action type
	private static HashMap<Class<?>, Object> applyRules = new HashMap<Class<?>, Object>();
}
