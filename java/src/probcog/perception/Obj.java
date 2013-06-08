package probcog.perception;

import java.util.*;

public class Obj
{
    private int id;
    private PointCloud ptCloud;
    private ArrayList<String> labels;
    // private HashMap<FeatureCategory, ArrayList<Double> > features;
    private int segmentColor;

    public Obj()
    {
        id = 0; // XXX - NEED TO GET SEQUENTIAL IDS
        labels = new ArrayList<String>();
        // features = new HashMap<FeatureCategory, ArrayList<Double> >();
    }

    public Obj(PointCloud ptCloud)
    {
        id = 0; // XXX - NEED TO GET SEQUENTIAL IDS
        this.ptCloud = ptCloud;
        labels = new ArrayList<String>();
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

    public void setPoints(PointCloud ptClout)
    {
        this.ptCloud = ptCloud;
    }

    public PointCloud getPoints()
    {
        return ptCloud;
    }

    public void setVisColor(int color)
    {
        segmentColor = color;
    }

    public int getVisColor()
    {
        return segmentColor;
    }

}