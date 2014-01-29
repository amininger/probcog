package probcog.classify;

import java.util.*;

import april.jmat.*;

import probcog.perception.*;
import probcog.util.BoundingBox;

public class WeightFeatureExtractor
{
    public static ArrayList<Double> getFeatures(PointCloud cloud)
    {
    	ArrayList<Double> features = new ArrayList<Double>();
    	ArrayList<double[]> points = cloud.getPoints();
    	
    	if(points.size() == 0){
    		features.add(0.0);
    	} else {
    		BoundingBox bbox = BoundingBox.getMinimalXY(cloud.getPoints());
    		features.add(bbox.volume() * 1000);
    	}

		return features;
    }
}
