package soargroup.mobilesim.sim.actions;

import soargroup.mobilesim.sim.*;

public class PutDownFloorAction extends PutDownAction {
	public PutDownFloorAction(RosieSimObject object){
		super(object);
	}

	public String toString(){
		return "PutDownFloor(" + object.toString() + ")";
	}
}
