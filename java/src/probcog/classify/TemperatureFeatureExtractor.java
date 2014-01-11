package probcog.classify;

import java.util.*;

import april.jmat.*;

import probcog.perception.*;

public class TemperatureFeatureExtractor
{
    public static ArrayList<Double> getFeatures(PointCloud cloud)
    {
        ArrayList<Double> features = new ArrayList<Double>();
        double[] hsv = ColorFeatureExtractor.avgHSV(cloud.getPoints());
        features.add(hsv[0]);

		return features;
    }
}
