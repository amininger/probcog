package probcog.classify;

import java.util.*;

import probcog.perception.PointCloud;

public class ShapeFeatureExtractor
{

    public static ArrayList<Double> getFeatures(PointCloud cloud)
    {
        ArrayList<double[]> canonical = cloud.getCanonical();
        ArrayList<double[]> face = cloud.isolateTopFace(canonical);
        ArrayList<double[]> flat = cloud.flattenXY(face);
        ArrayList<double[]> normalized = cloud.normalize(flat);
        ArrayList<Double> features = PCA.getFeatures(normalized, 7);
       
        return features;
    }
}
