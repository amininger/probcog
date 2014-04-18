package probcog.sim;

import probcog.classify.WeightFeatureExtractor;
import probcog.classify.Features.FeatureCategory;
import probcog.perception.Obj;
import april.jmat.LinAlg;
import april.sim.SimWorld;

public class SimScale extends SimLocation implements ISimEffector{

	public SimScale(SimWorld sw) {
		super(sw);
	}

	@Override
	public void checkObject(Obj obj) {
		double[] diff = LinAlg.subtract(LinAlg.matrixToXyzrpy(T), obj.getPose());
		double dx = Math.abs(diff[0]);
		double dy = Math.abs(diff[1]);
		if(dx < lwh[0]/2*scale && dy < lwh[1]/2*scale){
			// Object is over center of scale, weigh it!
			obj.addFeatures(FeatureCategory.WEIGHT, WeightFeatureExtractor.getFeatures(obj.getPointCloud()));
		}
	}
}
