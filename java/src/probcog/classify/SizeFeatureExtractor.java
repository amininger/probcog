package probcog.classify;

import java.util.*;

import april.jmat.*;

import probcog.perception.*;

public class SizeFeatureExtractor
{
    // Slightly improved (???) size features
    public static ArrayList<Double> getFeatures(PointCloud cloud)
    {
        ArrayList<double[]> points = cloud.getPoints();
        ArrayList<Double> features = new ArrayList<Double>();

        // Now extract the appropriate values
		if(points.size() == 0){
			features.add(0.0);
			return features;
		}

        // Feature: average distance from the mean
        double[] mean = cloud.getCentroid();
		double distSum = 0;
		for(double[] pt : points){
            double[] pt3 = new double[]{pt[0], pt[1], pt[2]};
			distSum += LinAlg.distance(pt3, mean);
		}
		distSum /= points.size();

		features.add(distSum);

		return features;
    }
}
