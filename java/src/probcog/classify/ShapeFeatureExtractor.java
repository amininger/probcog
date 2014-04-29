package probcog.classify;

import java.util.*;

import probcog.perception.PointCloud;

public class ShapeFeatureExtractor
{

	public static int FPS = 9;
    public static ArrayList<Double> getFeatures(PointCloud cloud)
    {
    	ArrayList<double[]> lower = PointCloud.isolateTopFace(cloud.getPoints());
    	//ArrayList<double[]> top = cloud.isolateTopFace(cloud.getPoints());
        ArrayList<double[]> canonical = PointCloud.getCanonical(lower);//(new PointCloud(top)).getCanonical();
        //ArrayList<double[]> face = cloud.isolateTopFace(canonical);
        ArrayList<double[]> flat = PointCloud.flattenXY(canonical);
        ArrayList<double[]> normalized = PointCloud.normalize(flat);
        ArrayList<Double> features = PCA.getFeatures(normalized, 7);

       
        return features;
    }
}
