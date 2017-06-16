package probcog.sim;

import java.io.IOException;

import probcog.classify.WeightFeatureExtractor;
import probcog.classify.Features.FeatureCategory;
import probcog.perception.Obj;
import april.jmat.LinAlg;
import april.sim.SimWorld;
import april.util.StructureReader;

public class SimStove extends SimLocation implements ISimEffector{

	public SimStove(SimWorld sw) {
		super(sw);
	}
	
	@Override
	public void checkObject(Obj obj){
		SimObjectPC simObj = obj.getSourceSimObjectPC();
		if(simObj == null || simObj instanceof SimLocation || !(simObj instanceof ISimStateful)){
			return;
		}

		// Door must be closed
		if(!currentState.containsKey(SoarConcepts.DOOR) || !currentState.get(SoarConcepts.DOOR).equals(SoarConcepts.CLOSED)){
			return;
		}
		// Heat must be on
		if(!currentState.containsKey(SoarConcepts.ACTIVATION) || !currentState.get(SoarConcepts.ACTIVATION).equals(SoarConcepts.ON)){
			return;
		}

		double[] diff = LinAlg.subtract(xyzrpy, obj.getPose());
		double dx = Math.abs(diff[0]);
		double dy = Math.abs(diff[1]);
		double[] dims = getScaledDims();
		if(dx < dims[0]/2 && dy < dims[1]/2){
			// Center of object is over the stove, cook it!
			((ISimStateful)simObj).setState("cooking", "true");
		}
	}
	
	/** Restore state that was previously written **/
	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		
		if(!currentState.containsKey(SoarConcepts.DOOR)){
			this.addNewState(SoarConcepts.DOOR, new String[]{SoarConcepts.CLOSED, SoarConcepts.OPEN});
		}
		if(!currentState.containsKey(SoarConcepts.ACTIVATION)){
			this.addNewState(SoarConcepts.ACTIVATION, new String[]{SoarConcepts.OFF, SoarConcepts.ON});
		}
    }
}
