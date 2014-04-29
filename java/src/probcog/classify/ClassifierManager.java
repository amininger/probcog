package probcog.classify;

import java.util.*;
import java.io.*;

import april.config.*;
import april.util.*;

import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.*;
import probcog.perception.Obj;

public class ClassifierManager
{
    // Undo/redo functionality
    Object stateLock = new Object();
    private LinkedList<StackEntry> undoStack = new LinkedList<StackEntry>();
    private LinkedList<StackEntry> redoStack = new LinkedList<StackEntry>();
    private class StackEntry
    {
        public CPoint point;
        public FeatureCategory cat;
        public String action;

        // Only used when reading in
        public StackEntry()
        {

        }

        public StackEntry(CPoint point_, FeatureCategory cat_, String action_)
        {
            point = point_;
            cat = cat_;
            action = action_;
        }

        public void write(StructureWriter outs) throws IOException
        {
            outs.writeString(point.label);
            outs.writeDoubles(point.coords);
            outs.writeString(cat.name());
            outs.writeString(action);
        }

        public void read(StructureReader ins) throws IOException
        {
            String label = ins.readString();
            double[] coords = ins.readDoubles();
            point = new CPoint(label, coords);

            cat = FeatureCategory.valueOf(ins.readString());

            action = ins.readString();
        }
    }

    // The classfiers
	private HashMap<FeatureCategory, Classifier> classifiers;

    public ClassifierManager()
    {
    }

	public ClassifierManager(Config config)
    {
        addClassifiers(config);
	}

    public void addClassifiers(Config config)
    {
        String colorDataFile = null, shapeDataFile = null, sizeDataFile = null;
        colorDataFile = config.getString("training.color_data");
        shapeDataFile = config.getString("training.shape_data");
        //sizeDataFile = config.getString("training.size_data");

		classifiers = new HashMap<FeatureCategory, Classifier>();

        // New classification code
        // For now, manually specified parameters on weight
        GKNNClassifier colorKNN = new GKNNClassifier(10, 0.1);
        colorKNN.setDataFile(colorDataFile);
        classifiers.put(FeatureCategory.COLOR, colorKNN);

        GKNNClassifier shapeKNN = new GKNNClassifier(10, .05);      // XXX Untested parameter
        shapeKNN.setDataFile(shapeDataFile);
        classifiers.put(FeatureCategory.SHAPE, shapeKNN);

        GKNNClassifier sizeKNN  = new GKNNClassifier(5, .00003);      // XXX Needs revisiting, both in terms of
        sizeKNN.setDataFile(sizeDataFile);      // XXX parameter and classification

        classifiers.put(FeatureCategory.SIZE, sizeKNN);
        
        GKNNClassifier weightKNN = new GKNNClassifier(1, .015);
        classifiers.put(FeatureCategory.WEIGHT, weightKNN);
        
        GKNNClassifier temperatureKNN = new GKNNClassifier(1, .01);
        classifiers.put(FeatureCategory.TEMPERATURE, temperatureKNN);

        reloadData();
    }

    public HashMap<FeatureCategory,Classifications> classifyAll(Obj objectToClassify)
    {
        HashMap<FeatureCategory,Classifications> results = new HashMap<FeatureCategory,Classifications>();
        for(FeatureCategory fc : classifiers.keySet()){
        	Classifications c = classify(fc, objectToClassify);
        	if(c != null){
        		results.put(fc, c);
        	}
        }

        return results;
    }

    public Classifications classify(FeatureCategory cat, Obj objToClassify)
    {
    	Classifier classifier = classifiers.get(cat);
    	ArrayList<Double> features;
    	if(Features.isVisualFeature(cat)){
    		features = Features.getFeatures(cat, objToClassify.getPointCloud());
    		objToClassify.addFeatures(cat, features);
    	} else {
    		features = objToClassify.getFeatures(cat);
    	}

		if(classifier == null || features == null){
		    return null;
		}
	
		Classifications classifications;
		synchronized (stateLock) {
		    classifications = classifier.classify(features);
		    objToClassify.addClassifications(cat, classifications);
		}
		return classifications;
    }

    public void addDataPoint(FeatureCategory cat, ArrayList<Double> features, String label){
		Classifier classifier = classifiers.get(cat);
		synchronized(stateLock){
		    if(cat != FeatureCategory.SHAPE){
			    CPoint point = new CPoint(label, features);
			    StackEntry entry = new StackEntry(point, cat, "ADD");
			    classifier.add(point);
			    undoStack.add(entry);
		    } else {
		    	int fps = 7; 
		    	double max = 0;
		    	for(int i = 2*fps; i < 3*fps; i++){
		    		max = Math.max(features.get(i), max);
		    	}
		    	System.out.println("");
		    	for(int i = 0; i < 2; i++){
		    		for(int j = 0; j < 2; j++){
		    			CPoint point = new CPoint(label, features);
		    			StackEntry entry = new StackEntry(point, cat, "ADD");
		    			classifier.add(point);
		    			undoStack.add(entry);
//		    			point.print();
		    			// Flip horizontally
		    			for(int f = 0; f < fps; f++){
		    				double hmax = features.get(1*fps+f);
		    				double hmin = features.get(3*fps+f);
		    				features.set(1*fps+f, 1.0 - hmin);
		    				features.set(3*fps+f, 1.0 - hmax);
		    			}
		    		}
		    		// Flip vertically
		    		for(int f = 0; f < fps; f++){
		    			double vmin = features.get(0*fps+f);
		    			double vmax = features.get(2*fps+f);
		    			features.set(0*fps+f, max - vmax);
		    			features.set(2*fps+f, max - vmin);
		    		}
		    	}
		    }
		}
    }

    public void clearData(){
	for(Classifier classifier : classifiers.values()){
	    synchronized(stateLock){
		classifier.clearData();
	    }
	}
    }

    public void reloadData(){
	for(Classifier classifier : classifiers.values()){
	    synchronized(stateLock){
		classifier.clearData();
		classifier.loadData();
	    }
	}
    }

    public boolean hasUndo()
    {
        return undoStack.size() > 0;
    }

    public boolean hasRedo()
    {
        return redoStack.size() > 0;
    }

    /** Undo function. Undoes the last action taken by the user */
    public void undo()
    {
        synchronized (stateLock) {
            if (undoStack.size() < 1)
                return;
            StackEntry entry = undoStack.pollLast();

            if (entry.action.equals("ADD")) {
                classifiers.get(entry.cat).removeLast();
                redoStack.add(entry);
            } else {
                System.err.println("ERR: Unhandled undo case - "+entry.action);
            }
        }
    }

    /** Redo function. Takes the last undone action and redoes it */
    public void redo()
    {
        synchronized (stateLock) {
            if (redoStack.size() < 1)
                return;
            StackEntry entry = redoStack.pollLast();

            if (entry.action.equals("ADD")) {
                classifiers.get(entry.cat).add(entry.point);
                undoStack.add(entry);
            } else {
                System.err.println("ERR: Unhandled redo case - "+entry.action);
            }
        }
    }

    // XXX Might want to spawn a backup thread to do this...
    /** Write out a backup file of our current state. */
    public void writeState(String filename) throws IOException
    {
        synchronized (stateLock) {
            // As it stands, the undo/redo stacks possess all of the
            // information necessary to back up the entire system
            // state. Thus, the implementation of undo and redo
            // are directly connected with our ability to load from
            // this state
            StructureWriter outs = new TextStructureWriter(new BufferedWriter(new FileWriter(filename)));

            outs.writeString("undo");
            outs.writeInt(undoStack.size());
            outs.blockBegin();
            for (StackEntry entry: undoStack) {
                outs.blockBegin();
                entry.write(outs);
                outs.blockEnd();
            }
            outs.blockEnd();

            outs.writeString("redo");
            outs.writeInt(redoStack.size());
            outs.blockBegin();
            for (StackEntry entry: redoStack) {
                outs.blockBegin();
                entry.write(outs);
                outs.blockEnd();
            }
            outs.blockEnd();

            outs.close();
        }
    }

    /** Read in a backup file of our current state,
     *  resetting all state to match that of the file
     */
    public void readState(String filename) throws IOException
    {
        synchronized (stateLock) {
            // Reset state
            clearData();

            // Again, based on the premise than undo and redo work a certain way
            StructureReader ins = new TextStructureReader(new BufferedReader(new FileReader(filename)));

            String undoString = ins.readString();
            assert (undoString.equals("undo"));
            int undoSize = ins.readInt();
            ins.blockBegin();
            for (int i = 0; i < undoSize; i++) {
                ins.blockBegin();
                StackEntry entry = new StackEntry();
                entry.read(ins);

                if (entry.action.equals("ADD")) {
                    classifiers.get(entry.cat).add(entry.point);
                }

                undoStack.add(entry);

                ins.blockEnd();
            }
            ins.blockEnd();

            String redoString = ins.readString();
            assert (redoString.equals("redo"));
            int redoSize = ins.readInt();
            ins.blockBegin();
            for (int i = 0; i < redoSize; i++) {
                ins.blockBegin();
                StackEntry entry = new StackEntry();
                entry.read(ins);

                redoStack.add(entry);

                ins.blockEnd();
            }
            ins.blockEnd();

            ins.close();
        }
    }
}
