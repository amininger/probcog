package soargroup.mobilesim.sim.actions;

import soargroup.mobilesim.sim.*;

public class PlaceObject extends PutDown {
	public final String relation;
	public final RosieSimObject target;
	public PlaceObject(RosieSimObject object, String relation, RosieSimObject target){
		super(object);
		this.relation = relation;
		this.target = target;
	}

	public String toString(){
		return "PlaceObject(" + object + " " + relation + " " + target);
	}
}
