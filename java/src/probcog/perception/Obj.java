package probcog.perception;

import java.util.*;

import probcog.classify.Features.FeatureCategory;

public class Obj
{
    private int id;
    private PointCloud ptCloud;
    private double[][] bbox;
    private double[] centroid;
    private HashMap<FeatureCategory, HashSet<String>> labels;

    public Obj(boolean assignID)
    {
        if(assignID)
            id = nextID();

        labels = new HashMap<FeatureCategory, HashSet<String>>();
        bbox = new double[2][3];
        centroid = new double[3];
    }

    public Obj(boolean assignID, PointCloud ptCloud)
    {
        if(assignID)
            id = nextID();

        this.ptCloud = ptCloud;
        labels = new HashMap<FeatureCategory, HashSet<String>>();
        bbox = ptCloud.getBoundingBox();
        centroid = ptCloud.getCentroid();
    }

    public Obj(int id)
    {
        this.id = id;
        labels = new HashMap<FeatureCategory, HashSet<String>>();
        bbox = new double[2][3];
        centroid = new double[3];
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

    public void addLabel(FeatureCategory category, String label)
    {
        HashSet<String> allLabels;
        if(labels.containsKey(category))
            allLabels = labels.get(category);
        else
            allLabels = new HashSet<String>();

        allLabels.add(label);
        labels.put(category, allLabels);
    }

    public void addLabels(FeatureCategory category, HashSet<String> newLabels)
    {
        HashSet<String> allLabels;
        if(labels.containsKey(category))
            allLabels = labels.get(category);
        else
            allLabels = new HashSet<String>();

        allLabels.addAll(newLabels);
        labels.put(category, allLabels);
    }

    public void addLabels(FeatureCategory category, ArrayList<String> newLabels)
    {
        HashSet<String> allLabels;
        if(labels.containsKey(category))
            allLabels = labels.get(category);
        else
            allLabels = new HashSet<String>();

        for(String label : newLabels)
            allLabels.add(label);
        labels.put(category, allLabels);
    }

    public void addLabels(HashMap<FeatureCategory, HashSet<String>> newLabels)
    {
        for(FeatureCategory category : newLabels.keySet()){
            HashSet<String> allLabels;
            if(labels.containsKey(category))
                allLabels = labels.get(category);
            else
                allLabels = new HashSet<String>();

            allLabels.addAll(newLabels.get(category));
            labels.put(category, allLabels);
        }
    }

    public Set<String> getLabels(FeatureCategory category)
    {
        if(labels.containsKey(category)){
    		return labels.get(category);
    	}
        return new HashSet<String>();
    }

    public HashMap<FeatureCategory, HashSet<String>> getLabels()
    {
        return labels;
    }

    public void clearLabels()
    {
        labels = new HashMap<FeatureCategory, HashSet<String>>();
    }

    public void clearLabels(FeatureCategory category)
    {
        labels.put(category, new HashSet<String>());
    }

    // Increasing ids
    private static int idGen = 0;
    public static int nextID()
    {
        return idGen ++;
    }
}
