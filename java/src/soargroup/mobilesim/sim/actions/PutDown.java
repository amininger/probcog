package soargroup.mobilesim.sim.actions;

import soargroup.mobilesim.sim.*;

public class PutDown extends Action {
	public final RosieSimObject object;
	public PutDown(RosieSimObject object){
		this.object = object;
	}

	public String toString(){
		return "PutDown(" + object + ")";
	}
}
