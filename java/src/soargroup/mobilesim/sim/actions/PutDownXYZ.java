package soargroup.mobilesim.sim.actions;

import soargroup.mobilesim.sim.*;

public class PutDownXYZ extends PutDown {
	public final Double x, z, y;
	public PutDownXYZ(RosieSimObject object, double[] xyz){
		super(object);
		this.x = xyz[0];
		this.y = xyz[1];
		this.z = xyz[2];
	}

	public String toString(){
		return "PutDownXYZ(" + object + ", " + x + ", " + y + ", " + z + ")";
	}
}
