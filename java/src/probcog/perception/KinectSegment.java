package probcog.perception;

import java.awt.Color;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;

import probcog.arm.ArmStatus;
import probcog.sensor.KinectSensor;
import probcog.sensor.Sensor;
import probcog.sensor.SimKinectSensor;
import probcog.util.Util;
import april.config.Config;
import april.jmat.LinAlg;
import april.jmat.geom.GLine3D;
import april.sim.SimWorld;
import april.util.TimeUtil;
import april.util.UnionFindSimple;

public class KinectSegment implements Segmenter
{
    final static double COLOR_THRESH = .02;//30;
    final static double DISTANCE_THRESH = 0.01;
    final static double MIN_FROM_FLOOR_PLANE = .010; // XXX 0.015
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
        points = kinect.getAllXYZRGB();
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
        	segmentedObjects.add(new Obj(false, pc));
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

        for(PointCloud ptc : idToPoints.values())
            objects.add(ptc);

        return objects;
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
