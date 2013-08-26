package probcog.sim;

import java.io.IOException;

import april.jmat.LinAlg;
import april.sim.SimWorld;
import april.util.StructureReader;

public class SimStove extends SimLocation implements ISimEffector{

	public SimStove(SimWorld sw) {
		super(sw);
	}

	@Override
	public void checkObject(SimObjectPC obj) {
		// Door must be closed
		if(!currentState.containsKey("door") || !currentState.get("door").equals("closed")){
			return;
		}
		// Heat must be on
		if(!currentState.containsKey("heat") || !currentState.get("heat").equals("on")){
			return;
		}
		// Must be able to set the state of the object
		if(!(obj instanceof ISimStateful)){
			return;
		}
		
		double[] diff = LinAlg.subtract(LinAlg.matrixToXyzrpy(T), LinAlg.matrixToXyzrpy(obj.getPose()));
		double dx = Math.abs(diff[0]);
		double dy = Math.abs(diff[1]);
		if(dx < lwh[0] * scale && dy < lwh[1] * scale){
			// Center of object is over the stove, cook it!
			((ISimStateful)obj).setState("cooking", "true");
		}
	}
	
	/** Restore state that was previously written **/
	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		
		if(!currentState.containsKey("door")){
			this.addNewState("door", new String[]{"closed", "open"});
		}
		if(!currentState.containsKey("heat")){
			this.addNewState("heat", new String[]{"off", "on"});
		}
    }
}
