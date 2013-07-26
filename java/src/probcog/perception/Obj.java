package probcog.perception;

import java.awt.*;
import java.util.*;

import april.jmat.*;
import april.sim.SphereShape;
import april.sim.Shape;
import april.vis.*;

import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.*;

public class Obj
{
    private int id;

    // Point cloud information
    private PointCloud ptCloud;
    private double[][] bbox;
    private double[] centroid;

    // Labels and attributes of the object
    private HashMap<FeatureCategory, Classifications> labels;
    private HashMap<FeatureCategory, ArrayList<Double>> features;
    private HashMap<String, String[]> possibleStates;
    private HashMap<String, String> currentStates;

    // Visualization information
	private Shape shape;
	private VisChain model;
    private double[] pose;
    private boolean visible;

    public Obj(boolean assignID)
    {
        if(assignID)
            id = nextID();

        labels = new HashMap<FeatureCategory, Classifications>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        possibleStates = new HashMap<String, String[]>();
        currentStates = new HashMap<String, String>();
        bbox = new double[2][3];
        centroid = new double[3];
        ptCloud = new PointCloud();
    }

    public Obj(boolean assignID, PointCloud ptCloud)
    {
        if(assignID)
            id = nextID();

        this.ptCloud = ptCloud;

        labels = new HashMap<FeatureCategory, Classifications>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        possibleStates = new HashMap<String, String[]>();
        currentStates = new HashMap<String, String>();

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

        labels = new HashMap<FeatureCategory, Classifications>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        possibleStates = new HashMap<String, String[]>();
        currentStates = new HashMap<String, String>();

        bbox = new double[2][3];
        centroid = new double[3];
		pose = new double[6];
        shape = new SphereShape(.01);
        model = null;
        ptCloud = new PointCloud();
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
    public void setShape(Shape shape)
    {
        this.shape = shape;
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
    public void setVisObject(VisChain model)
    {
        this.model = model;
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

    public void addClassifications(FeatureCategory category, Classifications cs)
    {
        labels.put(category, cs);
    }

    public void addAllClassifications(HashMap<FeatureCategory, Classifications> allCS)
    {
        for(FeatureCategory fc : allCS.keySet())
            labels.put(fc, allCS.get(fc));
    }

    public Classifications getLabels(FeatureCategory category)
    {
        if(labels.containsKey(category)){
    		return labels.get(category);
    	}
        return new Classifications();
    }

    public HashMap<FeatureCategory, Classifications> getLabels()
    {
        return labels;
    }

    public categorized_data_t[] getCategoryData()
    {
        categorized_data_t[] cat_dat = new categorized_data_t[labels.size()];
        int j = 0;
        for (FeatureCategory fc: labels.keySet()) {
            cat_dat[j] = new categorized_data_t();
            cat_dat[j].cat = new category_t();
            cat_dat[j].cat.cat = Features.getLCMCategory(fc);
            Classifications cs = labels.get(fc);
            cs.sortLabels();    // Just to be nice
            cat_dat[j].len = cs.size();
            cat_dat[j].confidence = new double[cat_dat[j].len];
            cat_dat[j].label = new String[cat_dat[j].len];

            int k = 0;
            for (Classifications.Label label: cs.labels) {
                cat_dat[j].confidence[k] = label.weight;
                cat_dat[j].label[k] = label.label;
                k++;
            }
            
           
            j++;
        }
        return cat_dat;
    }
    
    public String[] getStates(){
    	String[] stateVals = new String[currentStates.size()]; 
    	int i = 0;
    	for(Map.Entry<String, String> stateVal : currentStates.entrySet()){
    		stateVals[i++] = stateVal.getKey() + "=" + stateVal.getValue();
        }
        return stateVals;
    }


    // ATTRIBUTES / STATES
    public void setPossibleStates(HashMap<String, String[]> possible)
    {
        possibleStates = possible;
    }

    public void setCurrentStates(HashMap<String, String> current)
    {
        currentStates = current;
    }

    public void setState(String stateName, String stateVal)
    {
        String[] states = possibleStates.get(stateName.toLowerCase());
        if(states == null || !Arrays.asList(states).contains(stateVal.toLowerCase())) {
        	// Either the stateName or stateVal is not recognized
            return;
        }
        currentStates.put(stateName.toLowerCase(), stateVal.toLowerCase());
    }



    // Increasing ids
    private static int idGen = 0;
    public static int nextID()
    {
        return idGen ++;
    }
}
