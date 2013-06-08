package probcog.perception;

import java.util.*;

import april.jmat.*;

public class PointCloud
{
    private ArrayList<double[]> pts;
    private double[] centroid;

    /** Initiallize a point cloud with an empty list of points. **/
    public PointCloud()
    {
        pts = new ArrayList<double[]>();
        centroid = null;
    }

    /** PointCloud can be initialized with a single point. **/
    public PointCloud(double[] point)
    {
        pts = new ArrayList<double[]>();
        pts.add(point);
        centroid = null;
    }


    public void addPoint(double[] pt)
    {
        pts.add(pt);
    }

    public void addPoints(ArrayList<double[]> newPts)
    {
        pts.addAll(newPts);
    }

    public ArrayList<double[]> getPoints()
    {
        return pts;
    }

    /** Returns the average location of all the points in the pointcloud. **/
    public double[] getCentroid()
    {
        if(centroid == null)
            computeCentroid();
        return centroid;
    }

    /** Calculate an axis-aligned bounding box of the points in the pointcloud.
     *  @return an array of the form {{minX, minY, minZ}, {maxX, maxY, maxZ}}
     **/
    public double[][] getBoundingBox()
    {
        double[][] bbox = new double[2][3];
        double[] min = new double[]{Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        double[] max = new double[]{-Double.MAX_VALUE, -Double.MAX_VALUE, -Double.MAX_VALUE};
        for(double[] p : pts){
            for(int i=0; i<min.length; i++){
                if(p[i]<min[i])
                    min[i] = p[i];
                if(p[i] > max[i])
                    max[i] = p[i];
            }
        }

        bbox[0] = min;
        bbox[1] = max;
        return bbox;
    }

    /** Translate the entire point cloud and update the centroid. **/
    public void translate(double[] xyz)
    {
        for(double[] p : pts)
            LinAlg.plusEquals(p, xyz);
        LinAlg.plusEquals(centroid, xyz);
    }


    /** Compute the average of all the points in the point cloud. Automatically
     *  sets result to centroid. **/
    private void computeCentroid()
    {
        if(pts.size() < 1){
            System.err.println("No points in point cloud - can't compute center");
            centroid = new double[3];
        }
        else{
            centroid = new double[pts.get(0).length];
            for(double[] p : pts){
                LinAlg.plusEquals(centroid, p);
            }

            for(int i=0; i<centroid.length; i++){
                centroid[i] /= (double) pts.size();
            }
        }
    }
}