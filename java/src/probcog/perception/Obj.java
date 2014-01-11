package probcog.perception;

import java.awt.Color;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import probcog.classify.Classifications;
import probcog.classify.Features;
import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.categorized_data_t;
import probcog.lcmtypes.category_t;
import probcog.sim.ISimStateful;
import probcog.sim.SimLocation;
import probcog.sim.SimObjectPC;
import probcog.util.BoundingBox;
import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimObject;
import april.vis.VisObject;
import april.vis.VzMesh;

public class Obj
{
    private int id;

    // Point cloud information
    private PointCloud ptCloud;
    private BoundingBox bbox;
    private double[] centroid;

    // Labels and attributes of the object
    private HashMap<FeatureCategory, Classifications> labels;
    private HashMap<FeatureCategory, ArrayList<Double>> features;
    private HashMap<String, String[]> possibleStates;
    private HashMap<String, String> currentStates;

    // Visualization information
	private Shape shape;
	private VisObject model;
    private double[] pose;

    private boolean confirmed = false;

    
    // If the object was created from a simulated object,
    //   This is the backwards pointer
    private SimObject sourceSimObj = null;
    private boolean visible = true; // XXX I think an object is always visible when first made?
    
    public Obj(boolean assignID)
    {
        if(assignID)
            id = nextID();

        labels = new HashMap<FeatureCategory, Classifications>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        possibleStates = new HashMap<String, String[]>();
        currentStates = new HashMap<String, String>();
        bbox = new BoundingBox();
        centroid = new double[3];
        ptCloud = new PointCloud();
        visible = true;
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
        visible = true;

        //double maxDim = Math.max(bbox[1][0]-bbox[0][0],
        //                         Math.max(bbox[1][1]-bbox[0][1], bbox[1][2]-bbox[0][2]));
        //shape = new SphereShape(maxDim);
        shape = new BoxShape(bbox.lenxyz[0],
                             bbox.lenxyz[1],
                             bbox.lenxyz[2]);

        int[] avgRGB = ptCloud.getAvgRGB();
        Color color = new Color(avgRGB[0], avgRGB[1], avgRGB[2]);
        model = bbox.getVis(new VzMesh.Style(color));   // XXX outline, instead?
    }

    public Obj(int id)
    {
        this.id = id;

        labels = new HashMap<FeatureCategory, Classifications>();
        features = new HashMap<FeatureCategory, ArrayList<Double>>();
        possibleStates = new HashMap<String, String[]>();
        currentStates = new HashMap<String, String>();

        bbox = new BoundingBox();
        centroid = new double[3];
		pose = new double[6];
        visible = true;
        shape = new BoxShape(.01, .01, .01);
        model = null;
        ptCloud = new PointCloud();
    }

    public boolean isConfirmed(){
    	return confirmed;
    }
    public void setConfirmed(boolean isConfirmed){
    	confirmed = isConfirmed;
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
    
    public void setSourceSimObject(SimObject obj){
    	this.sourceSimObj = obj;
    }
    public SimObject getSourceSimObject(){
    	return this.sourceSimObj;
    }
    public SimObjectPC getSourceSimObjectPC(){
    	if(this.sourceSimObj != null && this.sourceSimObj instanceof SimObjectPC){
    		return (SimObjectPC)sourceSimObj;
    	} else {
    		return null;
    	}
    }
    
    public void setPointCloud(PointCloud ptCloud)
    {
        this.ptCloud = ptCloud;
        bbox = ptCloud.getBoundingBox();
        centroid = ptCloud.getCentroid();
        pose = new double[]{centroid[0], centroid[1], centroid[2], 0, 0, 0};

		shape = new BoxShape(bbox.lenxyz[0], bbox.lenxyz[1], bbox.lenxyz[2]);

        shape = new BoxShape(bbox.lenxyz[0],
                             bbox.lenxyz[1],
                             bbox.lenxyz[2]);

        int[] avgRGB = ptCloud.getAvgRGB();
        Color color = new Color(avgRGB[0], avgRGB[1], avgRGB[2]);
        model = bbox.getVis(new VzMesh.Style(color));   // XXX Lines?

    }
    public PointCloud getPointCloud()
    {
        return ptCloud;
    }
    public void setBoundingBox(BoundingBox bbox)
    {
        this.bbox = bbox;
    }
    public BoundingBox getBoundingBox()
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
    public void setVisObject(VisObject model)
    {
        this.model = model;
    }
    public VisObject getVisObject()
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
    	SimObjectPC simObj = getSourceSimObjectPC();
    	if(simObj != null){
        	HashMap<FeatureCategory, String> simClassifications = simObj.getSimClassifications();
        	if(simClassifications.containsKey(category)){
        		cs = new Classifications();
        		cs.add(simClassifications.get(category), 1.0f);
        	}
    	}
        labels.put(category, cs);
    }

    public void addAllClassifications(HashMap<FeatureCategory, Classifications> allCS)
    {
    	for(Map.Entry<FeatureCategory, Classifications> e : allCS.entrySet()){
    		addClassifications(e.getKey(), e.getValue());
    	}
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
            
        	// Report the real classification(s)
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
            
            ArrayList<Double> fs = this.features.get(fc);
            if(fs == null || Features.isVisualFeature(fc)){
            	cat_dat[j].num_features = 0;
            	cat_dat[j].features = new double[0];
            } else {
	            cat_dat[j].num_features = fs.size();
	            cat_dat[j].features = new double[cat_dat[j].num_features];
	            for(int i = 0; i < cat_dat[j].num_features; i++){
	            	cat_dat[j].features[i] = fs.get(i);
	            }
            }
           
            j++;
        }
        return cat_dat;
    }
    
    public String[] getStates(){
    	if(sourceSimObj == null || !(sourceSimObj instanceof ISimStateful)){
    		return new String[0];
    	}
    	String[][] currentState = ((ISimStateful)sourceSimObj).getCurrentState();
    	String[] stateVals = new String[currentState.length]; 
    	for(int i = 0; i < currentState.length; i++){
    		stateVals[i] = currentState[i][0] + "=" + currentState[i][1];
    	}
        return stateVals;
    }

    // Increasing ids
    private static int idGen = 1;
    public static int nextID()
    {
        return idGen ++;
    }
    public static void idAssigned(int id){
    	if(id >= idGen){
    		idGen = id+1;
    	}
    }
}
