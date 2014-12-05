package probcog.sim;

import probcog.classify.TemperatureFeatureExtractor;
import probcog.classify.WeightFeatureExtractor;
import probcog.classify.Features.FeatureCategory;
import probcog.perception.Obj;
import april.jmat.LinAlg;
import april.sim.SimWorld;

public class SimThermometer extends SimLocation implements ISimEffector{

	public SimThermometer(SimWorld sw) {
		super(sw);
	}

	@Override
	public void checkObject(Obj obj) {
		double[] diff = LinAlg.subtract(obj.getPose(), xyzrpy);
		double[] dims = getScaledDims();
		if(Math.abs(diff[0]) < dims[0]/2 && Math.abs(diff[1]) < dims[1]/2 && diff[2] > dims[2]/2){
			// Object is over center of thermometer, report its temperature
			obj.addFeatures(FeatureCategory.TEMPERATURE, 
					TemperatureFeatureExtractor.getFeatures(obj.getPointCloud()));
		}
	}
}
