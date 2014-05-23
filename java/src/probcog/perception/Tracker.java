package probcog.perception;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import lcm.lcm.*;

import probcog.arm.*;
import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.*;
import probcog.sensor.*;
import probcog.sim.ISimEffector;
import probcog.sim.ISimStateful;
import probcog.sim.SimLocation;
import probcog.sim.SimObjectPC;
import probcog.util.*;

public class Tracker
{
    static LCM lcm = LCM.getSingleton();
    // Classification stuff
    ClassifierManager classyManager;

    // Soar stuff
    private Object soarLock;
    private soar_objects_t soar_lcm;

    // Arm Stuff
    private Object armLock = new Object();
    private robot_command_t robot_cmd;
    
    private Object locLock = new Object();
    
    private Segmenter segmenter;
    private ArmCommandInterpreter armInterpreter;

    // World state, as decided by Soar and the perception system
    private SimWorld world;
    public Object stateLock;
    HashMap<Integer, Obj> worldState;

    private boolean perfectSegmentation;
    public static boolean SHOW_TIMERS = false;//true; //AM: if true then print statements are produced that show timing information

    public double fps = 0;
    ArrayList<Double> frameTimes = new ArrayList<Double>();
    int frameIdx = 0;
    final int frameTotal = 10;
    
    public Tracker(Config config_, Boolean physicalKinect, Boolean perfectSegmentation, SimWorld world) throws IOException{


        this.world = world;
        this.perfectSegmentation = perfectSegmentation;
        worldState = new HashMap<Integer, Obj>();
        classyManager = new ClassifierManager(config_);
        armInterpreter = new ArmCommandInterpreter(config_, false);  // Debug off

        if(physicalKinect) {
            segmenter = new KinectSegment(config_);
        }
        else if(perfectSegmentation){
            segmenter = new SimKinectSegment(config_, world);
        }
        else {
        	segmenter = new KinectSegment(config_, world);
        }

        ArrayList<Obj> locations = createImaginedObjects(world, true);
        for(Obj ob : locations) {
            worldState.put(ob.getID(), ob);
        }

        stateLock = new Object();
        soarLock = new Object();
        soar_lcm = null;

        new ListenerThread().start();
        new TrackingThread().start();
    }
    
    public HashMap<Integer, Obj> getWorldState()
    {
    	synchronized(stateLock){
    		return worldState;
    	}
    }

    // Returns null if object not found
    public Obj getObject(Integer id)
    {
    	synchronized(stateLock){
    		return worldState.get(id);
    	}
    }
    
    private int matchObject(Obj obj, HashMap<Integer, Obj> candidates){
        // Iterate through our existing objects. If there is an object
        // sharing any single label within a fixed distance of an existing
        // object, take the existing object's ID. If the ID has already
        // been taken by another object, take a new ID
        double thresh = 0.02;
        double overlapThresh = .04;
        
    	double objVol = obj.getBoundingBox().volume();
    	double maxOverlapped = -1;
        int maxID = -1;
        
       // double minDist = Double.MAX_VALUE;
        for (Obj cand: candidates.values()) {
            double candVol = cand.getBoundingBox().volume();
            double iVol = BoundingBox.estimateIntersectionVolume(obj.getBoundingBox(), cand.getBoundingBox(), 8);
            if(iVol == 0){
            	continue;
            }

            double overlapped = iVol/objVol * iVol/candVol;
            if(overlapped > overlapThresh && overlapped >= maxOverlapped){
//            	System.out.println("== NEW BEST ==");
//            	System.out.println("  NEW VOL:   " + objVol);
//            	System.out.println("  SOAR VOL:  " + candVol);
//            	System.out.println("  INTERSXN:  " + iVol);
//            	System.out.println("  OVERLAP:   " + overlapped);
            	maxOverlapped = overlapped;
            	maxID = cand.getID();
            }
        }
        return maxID;
    }
    
    private void getBestMapping(HashMap<Obj, Integer> curMapping, ArrayList<Obj> objList, HashMap<Integer, Obj> candidates){
    	double OVERLAP_THRESH = .04;
    	
    	boolean[] objAssigned = new boolean[objList.size()];
    	int i = 0;
    	for(Obj obj : objList){
    		objAssigned[i++] = false;
    	}
    	
    	ArrayList<Obj> candList = new ArrayList<Obj>();
    	boolean[] candAssigned = new boolean[candidates.size()];
    	i = 0;
    	for(Obj cand : candidates.values()){
    		candList.add(cand);
    		candAssigned[i++] = false;
    	}
    	
    	class MapScore{
    		public MapScore(int o1, int o2, double sc)
    			{ obj1 = o1;  obj2 = o2;  score = sc; }
    		
    		public Double score;
    		public int obj1;
    		public int obj2;
    	};
    	Comparator<MapScore> MapScoreComparator = new Comparator<MapScore>(){
			public int compare(MapScore arg0, MapScore arg1) {
				return arg0.score.compareTo(arg1.score);
			}
    	};
    	
    	PriorityQueue<MapScore> queue = new PriorityQueue<MapScore>(10, MapScoreComparator);
    	for(i = 0; i < objList.size(); i++){
    		Obj obj = objList.get(i);
        	double objVol = obj.getBoundingBox().volume();
    		for(int j = 0; j < candList.size(); j++){
    			Obj cand = candList.get(j);
                double candVol = cand.getBoundingBox().volume();
    			double iVol = BoundingBox.estimateIntersectionVolume(obj.getBoundingBox(), cand.getBoundingBox(), 8);	
    			double overlapped = iVol/objVol * iVol/candVol;
    			if(overlapped > OVERLAP_THRESH){
    				queue.add(new MapScore(i, j, overlapped));
    			}
    		}
    	}
    	
    	while(!queue.isEmpty()){
    		MapScore score = queue.poll();
    		if(objAssigned[score.obj1] || candAssigned[score.obj2]){
    			continue;
    		}
    		objAssigned[score.obj1] = true;
    		candAssigned[score.obj2] = true;
    		curMapping.put(objList.get(score.obj1), candList.get(score.obj2).getID());
    	}
    }
    

    public void compareObjects()
    {
        Tic tic = new Tic();
    	long time = TimeUtil.utime();


    	// Get Soar Objects
        HashMap<Integer, Obj> soarObjects = getSoarObjects();
        if(SHOW_TIMERS){
        	System.out.println("  GET SOAR OBJECTS: " + (TimeUtil.utime() - time));
        	time = TimeUtil.utime();
        	System.out.println("  GET VISIBLE OBJECTS");
        }

        ArrayList<Obj> visibleObjects;
        ArrayList<Obj> imagined;
        synchronized(locLock){
        	visibleObjects = getVisibleObjects();
        	imagined = createImaginedObjects(world, false);
        }
        // Get Visible Objects
        if(SHOW_TIMERS){
        	System.out.println("  GET VISIBLE OBJECTS: " + (TimeUtil.utime() - time));
        	time = TimeUtil.utime();
        }

        HashMap<Integer, Obj> previousFrame = new HashMap<Integer, Obj>();
//        for(Obj obj : soarObjects.values()){
//        	previousFrame.put(obj.getID(), obj);
//        }

        synchronized (stateLock) {
        	for(Obj obj : worldState.values()){
                previousFrame.put(obj.getID(), obj);
        	}
            worldState = new HashMap<Integer, Obj>();

            // Perfect segmentation, relies on the ID's inherent in the source Sim objects
            if(perfectSegmentation){
            	for(Obj obj : visibleObjects){
            		if(obj.getSourceSimObject() != null && obj.getSourceSimObject() instanceof SimObjectPC){
            			SimObjectPC simObj = ((SimObjectPC)obj.getSourceSimObject());
            			obj.setID(simObj.getID());
            			worldState.put(obj.getID(), obj);
            		}
            	}

                for(Obj o : imagined) {
                    worldState.put(o.getID(), o);
                }

                if(SHOW_TIMERS){
                	System.out.println("  TRACKING: " + (TimeUtil.utime() - time));
                	time = TimeUtil.utime();
                }
    			
                return;
            }

            // XXX - Need better matching code here.
            //
            // Iterate through our existing objects. If there is an object
            // sharing any single label within a fixed distance of an existing
            // object, take the existing object's ID. If the ID has already
            // been taken by another object, take a new ID
            
            HashMap<Obj, Integer> idMapping = new HashMap<Obj, Integer>();
            for(Obj obj : visibleObjects){
            	idMapping.put(obj, -1);
            }
            // Match all visible objects against the previous frame
            getBestMapping(idMapping, visibleObjects, previousFrame);

            // Any objects that also matched a soar object are removed before the next stage
            HashMap<Integer, Obj> unmatchedSoarObjects = (HashMap<Integer, Obj>)soarObjects.clone();
            ArrayList<Obj> unmatchedObjects = new ArrayList<Obj>();
            for(Map.Entry<Obj, Integer> e : idMapping.entrySet()){
            	if(e.getValue() != -1 && unmatchedSoarObjects.containsKey(e.getValue())){
            		Obj obj = e.getKey();
            		Obj soarObj = unmatchedSoarObjects.get(e.getValue());
            		if(BoundingBox.estimateIntersectionVolume(obj.getBoundingBox(), soarObj.getBoundingBox(), 8) < .000000001){
            			// Not actually a match, doesn't intersect soar object
            			e.setValue(-1);
            			unmatchedObjects.add(e.getKey());
            		} else {
            			unmatchedSoarObjects.remove(e.getValue());
            		}
            	} else {
            		unmatchedObjects.add(e.getKey());
            	}
            }
            
            // We match any objects not already matched to soar objects against
            // all the soar objects just to see if there is a match as well
            getBestMapping(idMapping, unmatchedObjects, unmatchedSoarObjects);
            
            for(Map.Entry<Obj, Integer> e : idMapping.entrySet()){
            	Obj newObj = e.getKey();
            	Integer id = e.getValue();
            	if(id == -1){
            		newObj.setID(Obj.nextID());
            	} else {
            		newObj.setID(id);
            		newObj.setConfirmed(true);
            	}
            	worldState.put(newObj.getID(), newObj);
            }
            

            
            
//            double thresh = 0.02;
//            double overlapThresh = .04;
//            for (Obj newObj: visibleObjects) {
//            	double newObjVol = newObj.getBoundingBox().volume();
//                boolean matched = false;
//                double maxOverlapped = -1;   // How much the object is overlapped by a soar object
//                int maxID = -1;
//                
//               // double minDist = Double.MAX_VALUE;
//                for (Obj soarObj: soarObjects) {
//                    if (idSet.contains(soarObj.getID()))
//                        continue;
//                    
//                    double soarObjVol = soarObj.getBoundingBox().volume();
//                    double iVol = BoundingBox.estimateIntersectionVolume(newObj.getBoundingBox(), soarObj.getBoundingBox(), 8);
//                    if(iVol == 0){
//                    	continue;
//                    }
//
//                    double overlapped = iVol/newObjVol * iVol/soarObjVol;
//                    if(overlapped > overlapThresh && overlapped >= maxOverlapped){
//                    	System.out.println("== NEW BEST ==");
//                    	System.out.println("  NEW VOL:   " + newObjVol);
//                    	System.out.println("  SOAR VOL:  " + soarObjVol);
//                    	System.out.println("  INTERSXN:  " + iVol);
//                    	System.out.println("  OVERLAP:   " + overlapped);
//                    	matched = true;
//                    	maxOverlapped = overlapped;
//                    	maxID = soarObj.getID();
//                    }
//
////                    double dist = LinAlg.distance(newObj.getCentroid(), soarObj.getCentroid());
////                    if(dist < thresh && dist < minDist){
////                        matched = true;
////                        minID = soarObj.getID();
////                        minDist = dist;
////                    }
//                }
//
//                if (matched) {
//                    newObj.setID(maxID);
//                    newObj.setConfirmed(true);
//                }
//                else {
//                    if(previousFrame.size() > 0){
//                    	matched = false;
//                    	maxOverlapped = -1;
//                    	maxID = -1;
////
////                        double threshOld = .01;
////                        boolean matchedOld = false;
////                        double minDistOld = Double.MAX_VALUE;
////                        int minIDOld = -1;
//
//                        for (Obj oldObj: previousFrame) {
//                            if (idSet.contains(oldObj.getID()))
//                                continue;
//                            
//                            double oldObjVol = oldObj.getBoundingBox().volume();
//                            double iVol = BoundingBox.estimateIntersectionVolume(newObj.getBoundingBox(), oldObj.getBoundingBox(), 8);
//                            if(iVol == 0){
//                            	continue;
//                            }
//
//                            double overlapped = newObjVol/iVol * oldObjVol/iVol;
//                            index            if(overlapped > overlapThresh && overlapped >= maxOverlapped){
//                            	matched = true;
//                            	maxOverlapped = overlapped;
//                            	maxID = oldObj.getID();
//                            }
//
////                            double dist = LinAlg.distance(newObj.getCentroid(), oldObj.getCentroid());
////                            if(dist < threshOld && dist < minDistOld){
////                                matchedOld = true;
////                                minIDOld = oldObj.getID();
////                                minDistOld = dist;
////                            }
//                        }
//                        if(matched) {
//                            newObj.setID(maxID);
//                        }
//                        else {
//                            newObj.setID(Obj.nextID());
//                        }
//                    }
//
//                    else {
//                        newObj.setID(Obj.nextID());
//                    }
//                }
//                idSet.add(newObj.getID());
//
//                worldState.put(newObj.getID(), newObj);
//            }

            adjustBoundingBoxes(new ArrayList<Obj>(worldState.values()));
            
            
            for(Obj o : imagined) {
                worldState.put(o.getID(), o);
            }
            if(SHOW_TIMERS){
            	System.out.println("  TRACKING: " + (TimeUtil.utime() - time));
            	time = TimeUtil.utime();
            }
        }
        
        double frameTime = tic.toc();
        if (frameTimes.size() < frameTotal) {
            frameTimes.add(frameTime);
        } else {
            frameTimes.set(frameIdx, frameTime);
            frameIdx = (frameIdx+1)%frameTotal;
        }
        calcFPS();
    }

    private void calcFPS()
    {
        double sum = 0;
        for (Double time: frameTimes)
        {
            sum += time;
        }
        sum /= frameTimes.size();

        fps = 1.0/sum;
    }


    /** Returns a list of objects : " + (TimeUtil.utime() - time))that the kinect sees on the table. The objects
     *  are returned as Obj's from the segmenter, and are passed to the
     *  classifiers. The resulting point clouds, their locations, and the
     *  classifications are returned.
     **/
    private ArrayList<Obj> getVisibleObjects()
    {
    	long time = TimeUtil.utime();

    	// Get points and segment to get visible objects
    	if(SHOW_TIMERS){
         	System.out.println("    POINT EXTRACTION + SEG");
        }
        ArrayList<Obj> visibleObjects = segmenter.getSegmentedObjects();
        if(SHOW_TIMERS){
        	System.out.println("    POINT EXTRACTION + SEG: " + (TimeUtil.utime() - time));
        	time = TimeUtil.utime();
        }

        for(SimObject so : world.objects){
            if(so instanceof ISimEffector) {
            	ISimEffector effector = (ISimEffector)so;
            	for(Obj obj : visibleObjects){
            		effector.checkObject(obj);
            	}
            }
        }

        
        // Classify all visible objects
        for(Obj obj : visibleObjects){
        	obj.addAllClassifications(classyManager.classifyAll(obj));
        }
        if(SHOW_TIMERS){
        	System.out.println("    CLASSIFICATION: " + (TimeUtil.utime() - time));
        }

        return visibleObjects;
    }

    /** Returns a list of Obj that Soar believes exists in the world based on
     *  their most recent lcm message. Obj have information such as their center
     *  and features they were classified with. They do not have point clouds.
     **/
    public HashMap<Integer, Obj> getSoarObjects()
    {
        HashMap<Integer, Obj> soarObjects = new HashMap<Integer, Obj>();

        synchronized(soarLock) {
            if(soar_lcm != null) {

                for(int i=0; i<soar_lcm.num_objects; i++) {
                    object_data_t odt = soar_lcm.objects[i];
                    Obj sObj = new Obj(odt.id);
                    double[] xyzrpy = odt.pos;
                    sObj.setCentroid(new double[]{xyzrpy[0], xyzrpy[1], xyzrpy[2]});
                    sObj.setBoundingBox(new BoundingBox(odt.bbox_dim, odt.bbox_xyzrpy));

                    for(int j=0; j<odt.num_cat; j++) {
                        categorized_data_t cat = odt.cat_dat[j];
                        FeatureCategory fc = Features.getFeatureCategory(cat.cat.cat);
                        Classifications cs = new Classifications();
                        for(int k=0; k<cat.len; k++)
                            cs.add(cat.label[k], cat.confidence[k]);

                        sObj.addClassifications(fc, cs);
                    }
                    soarObjects.put(sObj.getID(), sObj);
                }
            }
        }
        return soarObjects;
    }


    //////////////
    /// Deal with objects that we can't see through the kinect but want to
    /// pretend are there (like locations)
    public ArrayList<Obj> createImaginedObjects(SimWorld sw, boolean assignID)
    {
        ArrayList<Obj> imagined = new ArrayList<Obj>();
        for(SimObject so : sw.objects)
        {
            if(so instanceof SimLocation) {
                SimLocation sl = (SimLocation) so;
                Obj locObj = sl.getObj(assignID);
                imagined.add(locObj);
            }
        }
        return imagined;
    }
    
    private class CollisionStruct{
    	public Obj obj;
    	public double[] lenxyz;
    	public double[] scaledLenxyz;
    	public double[][] xyzrpy;
    	public CollisionStruct(Obj obj){
    		this.obj = obj;
    		xyzrpy = LinAlg.xyzrpyToMatrix(obj.getBoundingBox().xyzrpy);
    		lenxyz = LinAlg.copy(obj.getBoundingBox().lenxyz);
    		scaledLenxyz = LinAlg.copy(lenxyz);
    	}
    	public boolean collidesWith(CollisionStruct col){
    		return Collisions.collision(new BoxShape(scaledLenxyz), xyzrpy,
    				new BoxShape(col.scaledLenxyz), col.xyzrpy);
    	}
    	public void setScale(double[] scale){
    		LinAlg.scale(lenxyz, scale, scaledLenxyz);
    	}
    	public void updateObj(double[] scale){
    		obj.setShape(new BoxShape(scaledLenxyz));
    		LinAlg.copy(scaledLenxyz, obj.getBoundingBox().lenxyz);
    	}
    }
    
    double[] adjustOnDims(CollisionStruct c1, CollisionStruct c2, int[] dims){
    	double[] scale = new double[]{1, 1, 1};

    	// Simple binary search, finds it within 1%
    	double min = 0.001, max = 1.0;
    	for(int i = 0; i < 8; i++){
    		double s = (max + min)/2;
    		for(int dim : dims){
    			scale[dim] = s;
    		}
    		c1.setScale(scale);
    		c2.setScale(scale);
    		if(c1.collidesWith(c2)){
    			max = s;
    		} else {
    			min = s;
    		}
    	}
    	
    	for(int dim : dims){
    		scale[dim] = min * .98;
    	}
    	return scale;
    }
    
    private double adjustBoundingBoxOnAxis(CollisionStruct c1, CollisionStruct c2, int dim){
    	double[] scale = new double[]{1, 1, 1};
    	scale[dim] = 0.001;
    	
    	c1.setScale(scale);
    	if(c1.collidesWith(c2)){
    		// nothing we can do, return 1
    		return 1;
    	}
    	
    	// Simple binary search, finds it within 1%
    	double min = 0.001, max = 1.0;
    	for(int i = 0; i < 8; i++){
    		double s = (max + min)/2;
    		scale[dim] = s;
    		c1.setScale(scale);
    		if(c1.collidesWith(c2)){
    			max = s;
    		} else {
    			min = s;
    		}
    	}
    	scale[dim] = 1;
    	c1.setScale(scale);
    	
    	return min;
    }
    
    private double[] adjustBoundingBox(CollisionStruct c1, CollisionStruct c2){
    	double[] xyzrpy1 = c1.obj.getBoundingBox().xyzrpy;
    	double[] xyzrpy2 = c2.obj.getBoundingBox().xyzrpy;
    	double[][] rot = LinAlg.rollPitchYawToMatrix(new double[]{xyzrpy1[3], xyzrpy1[4], xyzrpy1[5]});
    	double[] dp = LinAlg.subtract(LinAlg.resize(xyzrpy2, 3), LinAlg.resize(xyzrpy1, 3));
    	
    	double bestScale = 0;
    	int bestDim = -1; 
    	for(int dim = 0; dim < 3; dim++){
    		double dimScale = adjustBoundingBoxOnAxis(c1, c2, dim);
    		if(dimScale != 1 && dimScale > bestScale){
    			bestScale = dimScale;
    			bestDim = dim;
    		}
    	}
    	
    	if(bestDim == -1){
    		return null;
    	}
    	double[] scaleAxis = LinAlg.resize(rot[bestDim], 3);
    	if(LinAlg.dotProduct(scaleAxis, dp) < 0){
    		bestScale *= -1;
    	}
    	double[] xyzs = LinAlg.resize(LinAlg.subtract(LinAlg.resize(xyzrpy1, 3), LinAlg.scale(scaleAxis, bestScale * c1.lenxyz[bestDim]/2)), 6);
    	for(int i = 0; i < 3; i++){
    		xyzs[3+i] = (i != bestDim ? 1 : (1 + bestScale)/2.0);
    	}
    	
    	return xyzs;
    }
    
//    private void adjustBoundingBoxes(ArrayList<Obj> objs){
//    	ArrayList<CollisionStruct> structs = new ArrayList<CollisionStruct>();
//    	for(Obj obj : objs){
//    		structs.add(new CollisionStruct(obj));
//    	}
//    	for(int i = 0; i < structs.size(); i++){
//    		CollisionStruct col1 = structs.get(i);
//    		for(int j = i+1; j < structs.size(); j++){
//    			CollisionStruct col2 = structs.get(j);
//    			if(!col1.collidesWith(col2)){
//    				continue;
//    			}
//    			
//    			double[] bestFit1 = adjustBoundingBox(col1, col2);
//    			double[] bestFit2 = adjustBoundingBox(col2, col1);
//    			
//    			boolean use1 = false;
//    			boolean use2 = false;
//    			
//    			if(bestFit1 == null && bestFit2 == null){
//    				continue;
//    			} else if(bestFit1 == null){
//    				use2 = true;
//    			} else if(bestFit2 == null){
//    				use1 = true;
//    			} else {
//    				double s1 = bestFit1[3] * bestFit1[4] * bestFit1[5];
//    				double s2 = bestFit2[3] * bestFit2[4] * bestFit2[5];
//    				if(s1 > s2){
//    					use1 = true;
//    				} else {
//    					use2 = true;
//    				}
//    			}
//    			
//    			if(use1){
//    				for(int d = 0; d < 3; d++){
//    					col1.obj.getBoundingBox().xyzrpy[d] = bestFit1[d];
//    					col1.obj.getBoundingBox().lenxyz[d] *= bestFit1[3+d];
//    				}
//    			} else if(use2) {
//    				for(int d = 0; d < 3; d++){
//    					col2.obj.getBoundingBox().xyzrpy[d] = bestFit2[d];
//    					col2.obj.getBoundingBox().lenxyz[d] *= (1 + bestFit2[3+d])/2.0;
//    				}
//    			}
//    		}
//    	}
//    			
//    			
//    			
////    			
////    			int[][] dimsToTry = new int[][]{
////    //					new int[]{0},		// x axis only
////    //					new int[]{1},		// y axis only 
////    					new int[]{2}, 		// z axis only
////    //					new int[]{0, 1}, 	// x and y axis
////    //					new int[]{0, 1, 2}  // x, y, and z axes
////    			};
////    			
////    			double bestScore = 0;
////    			double[] bestScale = new double[]{ 1, 1, 1};
////    			for(int[] dims : dimsToTry){
////    				double[] scale = adjustOnDims(col1, col2, dims);
////    				double score = scale[0] * scale[1] * scale[2];
////    				if(score > bestScore){
////    					bestScore = score;
////    					bestScale = scale;
////    				}
////    			}
////    			
////					if(bestScore > .1){
////    				col1.updateObj(bestScale);
////    				col2.updateObj(bestScale);
////					}
////    		}
//    }
    
    
    private boolean verticallyAligned(double[] pt, BoundingBox bbox){
    	double[] xyz = LinAlg.resize(bbox.xyzrpy, 3);
    	double[] rpy = new double[]{bbox.xyzrpy[3], bbox.xyzrpy[4], bbox.xyzrpy[5]};
    	double[] dp = LinAlg.subtract(pt, xyz);
    	double[][] rot = LinAlg.rollPitchYawToMatrix(rpy);
    	double px = LinAlg.dotProduct(LinAlg.resize(rot[0], 3), dp);
    	double py = LinAlg.dotProduct(LinAlg.resize(rot[1], 3), dp);
    	if(Math.abs(px) < bbox.lenxyz[0]/2 && Math.abs(py) < bbox.lenxyz[1]/2){
    		return true;
    	}
    	return false;
    }
    
    private void adjustBoundingBoxes(ArrayList<Obj> objs){
        // XXX HACK HACK HACK HACK HACK HACK HACK HACK
        double fudge = 0.0010; // Just for you, James
        for (int i = 0; i < objs.size(); i++) {
        	Obj obj0 = objs.get(i);
        	Shape shape0 = obj0.getShape();
        	BoundingBox bbox0 = obj0.getBoundingBox();
            for (int j = i+1; j < objs.size(); j++) {
            	Obj obj1 = objs.get(j);
            	Shape shape1 = obj1.getShape();
            	BoundingBox bbox1 = obj1.getBoundingBox();
                if (Collisions.collision(shape0, LinAlg.xyzrpyToMatrix(bbox0.xyzrpy),
                                         shape1, LinAlg.xyzrpyToMatrix(bbox1.xyzrpy)))
                {
                	if(!verticallyAligned(LinAlg.resize(bbox0.xyzrpy, 3), bbox1) && 
                			!verticallyAligned(LinAlg.resize(bbox1.xyzrpy, 3), bbox0)){
                		continue;
                	}
                	double c0 = bbox0.xyzrpy[2];		// Center
                	double h0 = bbox0.lenxyz[2]/2;	// Height (half)
                	
                	double c1 = bbox1.xyzrpy[2]; 	// Center
                	double h1 = bbox1.lenxyz[2]/2; 	// Height (half)
                	
                    // Adjust zlens in bounding box based on separation
                    // between centroids. Don't forget to update shape!
                	
                	double dz = Math.abs(c1 - c0);
                	// The adjustment is how much the bboxes needed to be backed up so the boxes are separated on z
                	double adjustment = h0 + h1 + fudge - dz;
                	if(adjustment > 0){
                		if(h0 + h1 < adjustment){
                			// Already too small to do anything about (or horizontally overlapping)
                			continue;
                		}
                		
                		double adj0 =  h0 / (h0 + h1) * adjustment;
                		double adj1 =  h1 / (h0 + h1) * adjustment;
                		bbox0.lenxyz[2] -= adj0;
                		bbox1.lenxyz[2] -= adj1; 
                		if(bbox0.xyzrpy[2] < bbox1.xyzrpy[2]){
                			bbox0.xyzrpy[2] -= adj0/2;
                			bbox1.xyzrpy[2] += adj1/2;
                		} else {
                			bbox0.xyzrpy[2] += adj0/2;
                			bbox1.xyzrpy[2] -= adj1/2;
                		}
                		
                		obj0.setPose(bbox0.xyzrpy);
                		obj1.setPose(bbox1.xyzrpy);

                        obj0.setShape(new BoxShape(bbox0.lenxyz[0],
                                                      bbox0.lenxyz[1],
                                                      bbox0.lenxyz[2]));
                        obj1.setShape(new BoxShape(bbox1.lenxyz[0],
                                                      bbox1.lenxyz[1],
                                                      bbox1.lenxyz[2]));
                        if (Collisions.collision(obj0.getShape(), LinAlg.xyzrpyToMatrix(bbox0.xyzrpy),
                                obj1.getShape(), LinAlg.xyzrpyToMatrix(bbox1.xyzrpy))){
                        	System.out.println("STILL A COLLISION");
                        	
                        }
                	}
                }
            }
        }
        // =========== END HACK ======================
    	
    }
    
    //////////////////////////////////////////////////////////////
    // Methods for interacting with the classifierManager (used through guis)
    public void clearClassificationData()
    {
        classyManager.clearData();
    }
    public void reloadClassificationData()
    {
        classyManager.reloadData();
    }
    public void undoClassification()
    {
        classyManager.undo();
    }
    public void redoClassification()
    {
        classyManager.redo();
    }
    public void addTraining(FeatureCategory category, ArrayList<Double> features, String label)
    {
        classyManager.addDataPoint(category, features, label);
    }
    public void writeClassificationState(String filename) throws IOException
    {
        classyManager.writeState(filename);
    }
    public boolean canUndoClassification()
    {
        return classyManager.hasUndo();
    }
    public boolean canRedoClassification()
    {
        return classyManager.hasRedo();
    }
    public Classifications getClassification(FeatureCategory category, Obj ob)
    {
        return classyManager.classify(category, ob);
    }

    /** Build up the object_data_t describing the observed objects
     *  in the world. Runs classifiers on the objects and builds
     *  the appropriate lcmtypes to return.
     */
    public object_data_t[] getObjectData()
    {
        long utime = TimeUtil.utime();
        ArrayList<object_data_t> objList = new ArrayList<object_data_t>();

        synchronized (stateLock) {
            for (Obj ob: worldState.values()) {
        		SimObject simObj = ob.getSourceSimObject();
            	if(simObj != null && simObj instanceof SimObjectPC && !((SimObjectPC)simObj).getVisible()){
            		continue;
        		}
            	if(!ob.isVisible()){
            		continue;
            	}
            	object_data_t od = new object_data_t();

                od.utime = utime;
                od.id = ob.getID();
                od.pos = ob.getPose();

                BoundingBox bbox = ob.getBoundingBox();
                od.bbox_dim = bbox.lenxyz;
                od.bbox_xyzrpy = bbox.xyzrpy;

                od.state_values = ob.getStates();
                od.num_states = od.state_values.length;

                categorized_data_t[] cat_dat = ob.getCategoryData();
                od.num_cat = cat_dat.length;
                od.cat_dat = cat_dat;
                
                objList.add(od);
            }
        }
        
        object_data_t[] objArray = objList.toArray(new object_data_t[objList.size()]);
        return objArray;
    }

    private void handlePerceptionCommand(perception_command_t command){
    	if(command.command.toUpperCase().contains("SAVE_CLASSIFIERS")){
    		try {
    			String[] args = command.command.split("=");
    			if(args.length > 1){
    				String filename = args[1] + ".cls";
    				classyManager.writeState(filename);
    			}
			} catch (IOException e) {
				e.printStackTrace();
			}
    	} else if(command.command.toUpperCase().contains("LOAD_CLASSIFIERS")){
    		try {
    			String[] args = command.command.split("=");
    			if(args.length > 1){
    				String filename = args[1] + ".cls";
    				classyManager.readState(filename);
    			}
			} catch (IOException e) {
				e.printStackTrace();
			}
    	} else if(command.command.toUpperCase().equals("CLEAR_CLASSIFIERS")){
    		classyManager.clearData();
    	}

    }

    // === Methods for interacting with the sensor(s) attached to the system === //
    // XXX This is kind of weird
    public ArrayList<Sensor> getSensors()
    {
        return segmenter.getSensors();
    }

    // Given a command from soar to set the state for an object,
    //   sets the state if a valid command
    private void processSetStateCommand(set_state_command_t setState){
    	SimObject src = null;
    	synchronized(stateLock){
    		Obj obj = worldState.get(setState.obj_id);
    		if(obj != null){
    			src = obj.getSourceSimObject();
    		}
    	}
    	if(src != null && src instanceof ISimStateful){
    		synchronized(locLock){
    			((ISimStateful)src).setState(setState.state_name, setState.state_val);
    		}
		}
    }

    /** Class that continually listens for messages from Soar about what objects
     *  it believes exists in the world. The received lcm message is stored so it
     *  can be used upon request.
     **/
    class ListenerThread extends Thread implements LCMSubscriber
    {
        LCM lcm = LCM.getSingleton();

        public ListenerThread()
        {
            lcm.subscribe("SOAR_OBJECTS", this);
            lcm.subscribe("ROBOT_COMMAND", this);
            lcm.subscribe("SET_STATE_COMMAND", this);
            lcm.subscribe("PERCEPTION_COMMAND", this);
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/60);
            }
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                messageReceivedEx(lcm, channel, ins);
            } catch (IOException ioex) {
                System.err.println("ERR: LCM channel -"+channel);
                ioex.printStackTrace();
            }
        }

        public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
                throws IOException
        {
            if (channel.equals("SOAR_OBJECTS")) {
                synchronized (soarLock) {
                    soar_lcm = new soar_objects_t(ins);
                }
            } else if (channel.equals("ROBOT_COMMAND")) {
                synchronized (armLock) {
                    robot_cmd = new robot_command_t(ins);
                    armInterpreter.queueCommand(robot_cmd);
                }
            } else if(channel.equals("SET_STATE_COMMAND")){
            	set_state_command_t setState = new set_state_command_t(ins);
            	processSetStateCommand(setState);
            } else if(channel.equals("PERCEPTION_COMMAND")){
            	perception_command_t command = new perception_command_t(ins);
            	handlePerceptionCommand(command);
            }
        }
    }

    /** Runs in the background, updating our knowledge of the scene */
    public class TrackingThread extends Thread
    {
        public void run()
        {
            while (true) {
            	long startTime = TimeUtil.utime();
            	if(SHOW_TIMERS){
            		System.out.println("------------------------------");
            		System.out.println("TRACKER");
            	}
                compareObjects();
                HashMap<Integer, Obj> objs = getWorldState();
                ArrayList<Obj> objsList = new ArrayList<Obj>();
                for(Obj o : objs.values())
                    objsList.add(o);
                synchronized (armLock) {
                    armInterpreter.updateWorld(objsList);
                }
    			
                if(SHOW_TIMERS){
                    System.out.println("TRACKER: " + (TimeUtil.utime() - startTime));
                }

                TimeUtil.sleep(1000/30);
            }
        }
    }
}
