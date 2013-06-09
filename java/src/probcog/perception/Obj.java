package probcog.perception;

import java.util.*;

public class Obj
{
    private int id;
    private PointCloud ptCloud;
    private ArrayList<String> labels;
    private double[][] bbox;
    private double[] centroid;
    // private HashMap<FeatureCategory, ArrayList<Double> > features;

    public Obj(boolean assignID)
    {
        if(assignID)
            id = nextID();

        labels = new ArrayList<String>();
        bbox = new double[2][3];
        centroid = new double[3];
        // features = new HashMap<FeatureCategory, ArrayList<Double> >();
    }

    public Obj(boolean assignID, PointCloud ptCloud)
    {
        if(assignID)
            id = nextID();

        this.ptCloud = ptCloud;
        labels = new ArrayList<String>();
        bbox = ptCloud.getBoundingBox();
        centroid = ptCloud.getCentroid();
        // features = new HashMap<FeatureCategory, ArrayList<Double> >();
    }

    public Obj(int id)
    {
        this.id = id;
        labels = new ArrayList<String>();
        bbox = new double[2][3];
        centroid = new double[3];
        // features = new HashMap<FeatureCategory, ArrayList<Double> >();
    }

    // SET AND GET CALLS
    public void setID(int id)
    {
        this.id = id;
    }

    public int getID()
    {
        return id;
    }

    public void setPointCloud(PointCloud ptClout)
    {
        this.ptCloud = ptCloud;
        bbox = ptCloud.getBoundingBox();
        centroid = ptCloud.getCentroid();
    }

    public PointCloud getPointCloud()
    {
        return ptCloud;
    }

    public void setBoundingBox(double[][] bbox)
    {
        this.bbox = bbox;
    }

    public double[][] getBoundingBox()
    {
        return bbox;
    }

    public void setCentroid(double[] centroid)
    {
        this.centroid = centroid;
    }

    public double[] getCentroid()
    {
        return centroid;
    }

    // Increasing ids
    private static int idGen = 0;
    public static int nextID()
    {
        return idGen ++;
    }
}
