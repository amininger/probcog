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
}
