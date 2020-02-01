package soargroup.mobilesim.sim.actions;

import soargroup.mobilesim.sim.*;

public class PutDownAction extends Action {
	public final RosieSimObject object;
	public PutDownAction(RosieSimObject object){
		this.object = object;
	}

	public String toString(){
		return "PutDown(" + object.toString() + ")";
	}
}
