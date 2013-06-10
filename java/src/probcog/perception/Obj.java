package probcog.perception;

import java.awt.*;
import java.util.*;

import april.jmat.*;
import april.sim.SphereShape;
import april.sim.Shape;
import april.vis.*;

import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;

public class Obj
{
    private int id;
    private PointCloud ptCloud;
    private double[][] bbox;
    private double[] centroid;
    private HashMap<FeatureCategory, HashSet<String>> labels;
    private HashMap<FeatureCategory, ArrayList<Double>> features;

    // Visualization information
	private Shape shape;
	private VisChain model;
    private double[] pose;
    private boolean visible;

    public Obj(boolean assignID)
    {
        if(assignID)
            id = nextID();

        labels = new HashMap<FeatureCategory, HashSet<String>>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        bbox = new double[2][3];
        centroid = new double[3];
    }

    public Obj(boolean assignID, PointCloud ptCloud)
    {
        if(assignID)
            id = nextID();

        this.ptCloud = ptCloud;
        labels = new HashMap<FeatureCategory, HashSet<String>>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        bbox = ptCloud.getBoundingBox();
        centroid = ptCloud.getCentroid();
        pose = new double[]{centroid[0], centroid[1], centroid[2], 0, 0, 0};

        double maxDim = Math.max(bbox[1][0]-bbox[0][0],
                                 Math.max(bbox[1][1]-bbox[0][1], bbox[1][2]-bbox[0][2]));
        shape = new SphereShape(maxDim);

        int[] avgRGB = ptCloud.getAvgRGB();
        Color color = new Color(avgRGB[0], avgRGB[1], avgRGB[2]);
        model = new VisChain(LinAlg.translate(centroid),
                             LinAlg.scale(bbox[1][0]-bbox[0][0],
                                          bbox[1][1]-bbox[0][1],
                                          bbox[1][2]-bbox[0][2]),
                             new VzBox(new VzMesh.Style(color)));
    }

    public Obj(int id)
    {
        this.id = id;
        labels = new HashMap<FeatureCategory, HashSet<String>>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        bbox = new double[2][3];
        centroid = new double[3];
		pose = new double[6];
        shape = new SphereShape(.01);
        model = null;
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

    public void setPose(double[] pose)
    {
        this.pose = pose;
    }
    public double[] getPose()
    {
        return pose;
    }
	public double[][] getPoseMatrix()
    {
		return LinAlg.xyzrpyToMatrix(pose);
	}
    public Shape getShape()
    {
        return shape;
    }
    public boolean isVisible()
    {
        return visible;
    }
    public void setVisible(Boolean visible)
    {
        this.visible = visible;
    }
    public VisChain getVisObject()
    {
        return model;
    }




    // FEATURES / CLASSIFICATIONS / LABELS
    public void addFeatures(FeatureCategory category, ArrayList<Double> vector)
    {
        features.put(category, vector);
    }

    public ArrayList<Double> getFeatures(FeatureCategory category)
    {
        return features.get(category);
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

    public HashSet<String> getLabels(FeatureCategory category)
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
