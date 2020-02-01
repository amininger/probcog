package soargroup.mobilesim.sim.actions;

import soargroup.mobilesim.sim.*;

public class PutDownFloor extends PutDown {
	public PutDownFloor(RosieSimObject object){
		super(object);
	}

	public String toString(){
		return "PutDownFloor(" + object + ")";
	}
}
