package probcog.classify;

import java.util.*;

import april.jmat.*;

import probcog.perception.*;

public class TemperatureFeatureExtractor
{
    public static ArrayList<Double> getFeatures(PointCloud cloud)
    {
    	// Temperature right now is based on color
    	// Where red is the highest (1) and blue/purple is the coldest (0)
        ArrayList<Double> features = new ArrayList<Double>();
        double hue = ColorFeatureExtractor.avgHSV(cloud.getPoints())[0];
        hue += 1.25/12.0; // Shift right so red is max and purple is min
        if(hue > 1){
        	hue -= 1;
        }

        features.add(Math.round(100*hue)/100.0);

		return features;
    }
}
