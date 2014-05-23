package probcog.perception;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;

import probcog.arm.ArmStatus;
import probcog.classify.ColorFeatureExtractor;
import probcog.sensor.KinectSensor;
import probcog.sensor.Sensor;
import probcog.sensor.SimKinectSensor;
import probcog.util.BoundingBox;
import probcog.util.ConvexHull;
import probcog.util.Polyhedron3D;
import probcog.util.Util;
import april.config.Config;
import april.jmat.LinAlg;
import april.jmat.geom.GLine3D;
import april.sim.SimWorld;
import april.util.TimeUtil;
import april.util.UnionFindSimple;

public class KinectSegment implements Segmenter
{
    final static double COLOR_THRESH = .025;//30;
    final static double DISTANCE_THRESH = 0.01;
    final static double MIN_FROM_FLOOR_PLANE = .015; // XXX 0.015
    final static double MIN_OBJECT_SIZE = 100;
    final static double RANSAC_PERCENT = .6;
    final static int RANSAC_ITERATIONS = 1000;

    private ArrayList<double[]> points;
    private double[] floorPlane;
    private int height, width;

    private ArrayList<Sensor> sensors = new ArrayList<Sensor>();
    private Sensor kinect;
    private ArmStatus arm;
    private double baseHeight, wristHeight;
    private ArrayList<Double> armWidths;
    private ArrayList<double[]> armPoints;

    // Set up some "Random" colors to draw the segments
    static int[] colors = new int[]{0xff3300CC, 0xff9900CC, 0xffCC0099, 0xffCC0033,
        0xff0033CC, 0xff470AFF, 0xff7547FF, 0xffCC3300,
        0xff0099CC, 0xffD1FF47, 0xffC2FF0A, 0xffCC9900,
        0xff00CC99, 0xff00CC33, 0xff33CC00, 0xff99CC00};

    public KinectSegment(Config config_) throws IOException
    {
    	this(config_, null);
    }
    
    public KinectSegment(Config config_, SimWorld world) throws IOException
    {
    	// Get stuff ready for moving arms
        arm = new ArmStatus(config_);
        baseHeight = arm.baseHeight;
        wristHeight = arm.wristHeight;
        armWidths = arm.getArmSegmentWidths();

        points = new ArrayList<double[]>();
        if(world == null){
            kinect = new KinectSensor(config_);
        } else {
        	kinect = new SimKinectSensor(world);
        }

        sensors.add(kinect);    // XXX
    }

    /** Get the most recent frame from the Kinect and segment it into point
     ** clouds of objects. Remove the floor plane and the segments that lie
     ** where the arm is expected to be.
     ** @return list of point clouds for each object segmented in the scene.
     **/
    public ArrayList<Obj> getSegmentedObjects()
    {
        if (!kinect.stashFrame())
            return new ArrayList<Obj>();

        width = kinect.getWidth();
        height = kinect.getHeight();

        // Get points from camera
        long time = TimeUtil.utime();
        points = kinect.getAllXYZRGB(true);
        if(Tracker.SHOW_TIMERS){
        	System.out.println("      TRACING: " + (TimeUtil.utime() - time));
        	time = TimeUtil.utime();
        }
        
        // Remove floor and arm points
        removeFloorAndArmPoints();
        if(Tracker.SHOW_TIMERS){
        	System.out.println("      REMOVE POINTS: " + (TimeUtil.utime() - time));
        	time = TimeUtil.utime();
        }
        
        // Do a union find to do segmentation
        ArrayList<PointCloud> pointClouds = unionFind();
        if(Tracker.SHOW_TIMERS){
        	System.out.println("      SEGMENTATION: " + (TimeUtil.utime() - time));
        	time = TimeUtil.utime();
        }
        
        ArrayList<Obj> segmentedObjects = new ArrayList<Obj>();
        for(PointCloud pc : pointClouds){
        	segmentedObjects.add(new Obj(false, pc.removeTopPoints(0.05)));
        }
        return segmentedObjects;
    }


    /** "Remove" points that are too close to the floor by setting them to
     ** empty arrays.
     ** @return whether a plane was found and points were removed
     **/
    private boolean removeFloorAndArmPoints()
    {
        // Only calculate the floor plane once
        if(floorPlane == null && points.size() > 0){
            System.out.println("Getting floor plane!");
            points = kinect.getAllXYZRGB(false);
            floorPlane = RANSAC.estimatePlane(points, RANSAC_ITERATIONS, RANSAC_PERCENT);
        }

        // Figure out where each of the joints are
        armPoints = arm.getArmPoints();

        // Remove points that are either on the floor plane, below the plane,
        // or along the arm's position
        for(int i=0; i<points.size(); i++){
            double[] point = points.get(i);
            double[] p = new double[]{point[0], point[1], point[2]};
            double distToPlane = RANSAC.pointToPlaneDist(p, floorPlane);
            if(distToPlane < MIN_FROM_FLOOR_PLANE ||
               distToPlane > wristHeight ||
               belowPlane(p, floorPlane) ||
               inArmRange(p))
            {
                points.set(i, new double[4]);
            }
        }
        return true;
    }

    /** union find- for each pixel, compare with pixels around it and merge if
     ** they are close enough. **/
    public ArrayList<PointCloud> unionFind()
    {
        ArrayList<PointCloud> objects = new ArrayList<PointCloud>();

        // Union pixels that are close together
        UnionFindSimple uf = new UnionFindSimple(points.size());
        for(int y=0; y<height; y++){
            for(int x=0; x<width; x++){
                int loc = y*width + x;
                double[] p1 = points.get(loc);

                // Look at neighboring pixels
                if(!almostZero(p1)){
                    int loc2 = y*width + x+1;
                    if (loc2>=0 && loc2<points.size() && (x+1)<width){
                        double[] p2 = points.get(loc2);
                        if(!almostZero(p2)
                           && (dist(p1, p2) < DISTANCE_THRESH
                               && colorDiff(p1[3], p2[3]) < COLOR_THRESH)){
                            uf.connectNodes(loc, loc2);
                        }
                    }
                    loc2 = (y+1)*width + x;
                    if (loc2>=0 && loc2<points.size() && (y+1)<height){
                        double[] p2 = points.get(loc2);
                        if(!almostZero(p2)
                           && (dist(p1, p2) < DISTANCE_THRESH
                               && colorDiff(p1[3], p2[3]) < COLOR_THRESH)){
                            uf.connectNodes(loc, loc2);
                        }
                    }
                }
            }
        }

        //collect data on all the objects segmented by the union find algorithm in the previous step
        HashMap<Integer, PointCloud> idToPoints = new HashMap<Integer, PointCloud>();
        for(int i = 0; i < points.size(); i++){
            double[] point = points.get(i);
            if(!almostZero(point) && uf.getSetSize(i) > MIN_OBJECT_SIZE){
                int ufID = uf.getRepresentative(i);
                PointCloud ptCloud = idToPoints.get(ufID);
                if(ptCloud == null)
                    ptCloud = new PointCloud();
                ptCloud.addPoint(point);
                idToPoints.put(ufID, ptCloud);
            }
        }
        
        HashMap<Integer, Integer> mappings = new HashMap<Integer, Integer>();
        ArrayList<PointCloud> clouds = new ArrayList<PointCloud>();
        ArrayList<BoundingBox> boxes = new ArrayList<BoundingBox>();
        int i = 0;
        for(PointCloud pc : idToPoints.values()){
        	clouds.add(pc);
        	boxes.add(pc.getBoundingBox());
        	mappings.put(i, i);
        	i++;
        }
        
//        for(i = 0; i < clouds.size(); i++){
//        	PointCloud c1 = clouds.get(i);
//        	BoundingBox bbox1 = boxes.get(i);
//        	for(int j = i+1; j < clouds.size(); j++){
//        		BoundingBox bbox2 = boxes.get(j);
//        		PointCloud c2 = clouds.get(j);
//        		if(BoundingBox.intersects(bbox1, bbox2, 1.1)){
//        			double[] hsv1 = ColorFeatureExtractor.avgHSV(c1.getPoints());
//        			double[] hsv2 = ColorFeatureExtractor.avgHSV(c2.getPoints());
//        			boolean merge = false;
//        			if(hsv1[0] > .8 && hsv2[0] > .8){
//        				merge = true;
//        			} else if(hsv1[0] <= .8 && hsv2[0] <= .8 && hsv1[0] > .65 && hsv2[0] > .65){
//        				merge = true;
//        			} else if(hsv1[0] <= .65 && hsv2[0] <= .65 && hsv1[0] > .54 && hsv2[0] > .54){
//        				merge = true;
//        			} else if(hsv1[0] <= .54 && hsv2[0] <= .54 && hsv1[0] > .48 && hsv2[0] > .48){
//        				merge = true;
//        			} else if(hsv1[0] <= .48 && hsv2[0] <= .48 && hsv1[0] > .2 && hsv2[0] > .2){
//        				merge = true;
//        			} else if(hsv1[0] <= .2 && hsv2[0] <= .2){
//        				merge = true;
//        			}
//        			if(merge){
////        				System.out.println("Merging " + i + " and " + j);
//        				int newSet = mappings.get(i);
//        				int oldSet = mappings.get(j);
//        				HashSet<Integer> setToMerge = new HashSet<Integer>();
//        				for(Map.Entry<Integer, Integer> e : mappings.entrySet()){
//        					if(e.getValue() == oldSet){
//        						setToMerge.add(e.getKey());
//        					}
//        				}
//        				for(Integer pcId : setToMerge){
//        					mappings.put(pcId, newSet);
//        				}
//        			}
//        		}
//        	}
//        }
        
        HashMap<Integer, PointCloud> mergedClouds = new HashMap<Integer, PointCloud>();
        for(Map.Entry<Integer, Integer> e : mappings.entrySet()){
        	
//        	System.out.println("Mapping " + e.getKey() + "=" + e.getValue());
        	PointCloud pc = clouds.get(e.getKey());
        	if(pc.getPoints().size() < MIN_OBJECT_SIZE){
        		continue;
        	}
        	if(mergedClouds.containsKey(e.getValue())){
        		mergedClouds.get(e.getValue()).addPoints(pc.getPoints());
        	} else {
        		mergedClouds.put(e.getValue(), pc); 
        	}
        }
        
        for(Map.Entry<Integer, PointCloud> e : mergedClouds.entrySet()){
        	objects.add(e.getValue());
        }
//        System.out.println(mergedClouds.size());
//
//        for(PointCloud ptc : idToPoints.values())
//            objects.add(ptc);

        return objects;
    }
    
    double calcContainment(ArrayList<double[]> points, Polyhedron3D poly, int numSamples){
    	double numContained = 0;
    	double[] pt = new double[3];
    	for(int i = 0; i < numSamples; i++){
    		double[] randPt = points.get((int)(Math.random() * points.size()));
    		pt[0] = randPt[0];
    		pt[1] = randPt[1];
    		pt[2] = randPt[2];
    		if(poly.contains(pt)){
    			numContained++;
    		}
    	}
    	return numSamples/numContained;
    }



    /** Determine whether a line is part of the arm, given a set of lines
     *  representing the positions of the joints.**/
    private boolean inArmRange(double[] p)
    {
        assert(armWidths.size() < armPoints.size());
        for (int i=0; i<armWidths.size(); i++) {
            double[] p0 = armPoints.get(i);
            double[] p1 = armPoints.get(i+1);
            GLine3D line = new GLine3D(p0, p1);
            double[] cp = line.pointOnLineClosestTo(p);

            if (cp[0] < Math.min(p0[0], p1[0]) ||
                cp[0] > Math.max(p0[0], p1[0]) ||
                cp[1] < Math.min(p0[1], p1[1]) ||
                cp[1] > Math.max(p0[1], p1[1]))
            {
                double d1 = LinAlg.distance(cp, p0);
                double d2 = LinAlg.distance(cp, p1);
                if (d1 < d2)
                    cp = p0;
                else
                    cp = p1;
            }

            if (LinAlg.distance(p, cp) < armWidths.get(i)) {
                return true;
            }
        }
        return false;
    }

    /** For determining whether a point is basically at the origin.**/
    private boolean almostZero(double[] p)
    {
        if((Math.abs(p[0]) < 0.0001) && (Math.abs(p[1]) < 0.0001) &&
           (Math.abs(p[2]) < 0.0001) && (Math.abs(p[3]) < 0.0001))
            return true;
        return false;
    }


    /** Check if a given point is on the other side of the ground plane. **/
    private boolean belowPlane(double[] p, double[] coef)
    {
        double eval = coef[0]*p[0] + coef[1]*p[1] + coef[2]*p[2] + coef[3];
        double aboveEval = coef[2]*10 + coef[3];
        if(eval < 0 && aboveEval > 0)
            return true;
        else if(eval > 0 && aboveEval < 0)
            return true;
        return false;
    }

    /** Get the difference in the z-direction of two pixels. **/
    private double dist(double[] p1, double[] p2)
    {
        double dx = p1[0]-p2[0];
        double dy = p1[1]-p2[1];
        double dz = p1[2]-p2[2];
        return Math.sqrt(dx*dx + dy*dy + dz*dz);
    }

    /** Find the Euclidean distance between two colors. **/
    private double colorDiff(double color1, double color2)
    {
        Color c1 = new Color((int)color1);
        Color c2 = new Color((int)color2);

        float[] hsv1 = Color.RGBtoHSB(c1.getRed(), c1.getGreen(), c1.getBlue(), null);
        float[] hsv2 = Color.RGBtoHSB(c2.getRed(), c2.getGreen(), c2.getBlue(), null);
        double diff = Math.abs((double)hsv1[0] - (double)hsv2[0]);

        return diff;
    }

    // === Provide access to the raw sensor === //
    // XXX This seems a bit hacky to provide...
    public ArrayList<Sensor> getSensors()
    {
        return sensors;
    }
}
