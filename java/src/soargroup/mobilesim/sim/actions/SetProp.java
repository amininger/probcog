package soargroup.mobilesim.sim.actions;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.sim.*;

public class SetProp extends Action {
	public final RosieSimObject object;
	public final String property;
	public final String value;

	public SetProp(RosieSimObject object, String property, String value){
		this.object = object;
		this.property = property;
		this.value = value;
	}

	// Constructs the SetProp action, using the appropriate subclass when needed
	//   (e.g. if property==activation1 and value=on1 will construct TurnOn)
	public static SetProp construct(RosieSimObject object, String property, String value){
		if(property.equals(RosieConstants.ACTIVATION) && value.equals(RosieConstants.ACT_ON)){
			return new TurnOn(object);
		} 
		else if(property.equals(RosieConstants.ACTIVATION) && value.equals(RosieConstants.ACT_OFF)){
			return new TurnOff(object);
		} 
		return new SetProp(object, property, value);
	}

	public String toString(){
		return "SetProp(" + object + ", " + property + "=" + value + ")";
	}

	// SetProp.TurnOn -> Turns on the object (e.g. a lightswitch)
	public static class TurnOn extends SetProp {
		public TurnOn(RosieSimObject object){
			super(object, RosieConstants.ACTIVATION, RosieConstants.ACT_ON);
		}

		public String toString(){
			return "SetProp.TurnOn(" + object + ")";
		}
	}

	// SetProp.TurnOff -> Turns off the object (e.g. a lightswitch)
	public static class TurnOff extends SetProp {
		public TurnOff(RosieSimObject object){
			super(object, RosieConstants.ACTIVATION, RosieConstants.ACT_OFF);
		}

		public String toString(){
			return "SetProp.TurnOff(" + object + ")";
		}
	}


}
