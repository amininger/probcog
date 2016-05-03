package probcog.old.sim;

import probcog.old.classify.TemperatureFeatureExtractor;
import probcog.old.classify.WeightFeatureExtractor;
import probcog.old.classify.Features.FeatureCategory;
import probcog.old.perception.Obj;
import april.jmat.LinAlg;
import april.sim.SimWorld;

public class SimThermometer extends SimLocation implements ISimEffector{

	public SimThermometer(SimWorld sw) {
		super(sw);
	}

	@Override
	public void checkObject(Obj obj) {
		double[] diff = LinAlg.subtract(LinAlg.matrixToXyzrpy(T), obj.getPose());
		double dx = Math.abs(diff[0]);
		double dy = Math.abs(diff[1]);
		if(dx < lwh[0] && dy < lwh[1] ){
			obj.addFeatures(FeatureCategory.TEMPERATURE, 
					TemperatureFeatureExtractor.getFeatures(obj.getPointCloud()));
		}
	}
}
