package probcog.perception;

import java.awt.*;
import java.io.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;

import probcog.sensor.*;

public class KinectSegment
{
    final static double COLOR_THRESH = .02;//30;
    final static double DISTANCE_THRESH = 0.01;
    final static double MIN_FROM_FLOOR_PLANE = .015;
    final static double MIN_OBJECT_SIZE = 100;
    final static double RANSAC_PERCENT = .2;
    final static int RANSAC_ITERATIONS = 2000;

    private ArrayList<double[]> points;
    private double[] floorPlane;
    private int height, width;

    private KinectSensor kinect;
    private double baseHeight, wristHeight;
    private ArrayList<Double> armWidths;

    // Set up some "Random" colors to draw the segments
    static int[] colors = new int[]{0xff3300CC, 0xff9900CC, 0xffCC0099, 0xffCC0033,
                                    0xff0033CC, 0xff470AFF, 0xff7547FF, 0xffCC3300,
                                    0xff0099CC, 0xffD1FF47, 0xffC2FF0A, 0xffCC9900,
                                    0xff00CC99, 0xff00CC33, 0xff33CC00, 0xff99CC00};



    public KinectSegment(Config color, Config ir, Config calib, Config armConfig)
    {
        // Get stuff ready for removing arms
        String armType = armConfig.getString("arm.arm_version",null);
        baseHeight = armConfig.getDouble("arm."+armType+".base_height",0);
        wristHeight = armConfig.getDouble("arm."+armType+".wrist_height",0);
        armWidths = new ArrayList<Double>();
        for (int i = 0;; i++) {
            double[] range = armConfig.getDoubles("arm."+armType+".r"+i+".range", null);
            double width = armConfig.getDouble("arm."+armType+".r"+i+".width", 0);
            if (range == null)
                break;

            armWidths.add(width);
        }
        armWidths.add(.05);
        armWidths.add(.05);

        try {
            kinect = new KinectSensor(color, ir, calib);
        } catch (IOException ioex) {
            System.err.println("ERR: Could not initialize KinectSensor");
            ioex.printStackTrace();
            System.exit(1);
        }
    }

    public ArrayList<PointCloud> getObjectPointClouds()
    {
        if (!kinect.stashFrame())
            return new ArrayList<PointCloud>();

        extractPoints(kinect);
        removeFloorAndArmPoints();
        return unionFind();
    }

    private void extractPoints(KinectSensor kinect)
    {
        points = new ArrayList<double[]>();
        height = kinect.getHeight();
        width = kinect.getWidth();

        for (int y = 0; y < height; y++) {
            for (int x = 0; x < width; x++) {
                double[] xyz = kinect.getXYZ(x,y);
                int rgb = kinect.getRGB(x,y); // Probably flipped?

                if (xyz == null)
                    continue;

                points.add(new double[]{xyz[0], xyz[1], xyz[2],rgb});
            }
        }
    }


    /** "Remove" points that are too close to the floor by setting them to
     ** empty arrays.
     ** @return whether a plane was found and points were removed
     **/
    private boolean removeFloorAndArmPoints()
    {
        // Only calculate the floor plane once
        if(floorPlane == null)
            floorPlane = RANSAC.estimatePlane(points, RANSAC_ITERATIONS, RANSAC_PERCENT);

        // Figure out where each of the joints are
        ArrayList<double[]> armPoints = new ArrayList<double[]>();
//        ArrayList<Joint> joints = tracker.arm.getJoints();
        double[][] xform = LinAlg.translate(0,0,baseHeight);
        armPoints.add(new double[3]);
        armPoints.add(LinAlg.matrixToXyzrpy(xform));
//        for (Joint j: joints) {
//            LinAlg.timesEquals(xform, j.getRotation());
//            LinAlg.timesEquals(xform, j.getTranslation());
//            armPoints.add(LinAlg.matrixToXyzrpy(xform));
//        }

        // Set up segments of the arm
        ArrayList<GLineSegment2D> armLines = new ArrayList<GLineSegment2D>();
        for(int i=1; i<armPoints.size(); i++){
            armLines.add(new GLineSegment2D(armPoints.get(i-1), armPoints.get(i)));
            double[] p1 = armPoints.get(i-1);
            double[] p2 = armPoints.get(i);
        }

        // Remove points that are either on the floor plane, below the plane,
        // or along the arm's position
        for(int i=0; i<points.size(); i++){
            double[] point = points.get(i);
            double[] p = new double[]{point[0], point[1], point[2]};
            double distToPlane = RANSAC.pointToPlaneDist(p, floorPlane);
            if(distToPlane < MIN_FROM_FLOOR_PLANE || distToPlane > wristHeight ||
               belowPlane(p, floorPlane) || inArmRange(armLines, p))
                points.set(i, new double[4]);
        }

        return true;
    }


    /** union find- for each pixel, compare with pixels around it and merge if
     ** they are close enough. **/
    public ArrayList<PointCloud> unionFind()
    {
        ArrayList objects = new ArrayList<PointCloud>();

        // Union pixels that are close together
        UnionFindSimple uf = new UnionFindSimple(points.size());
        for(int y=0; y<height; y++){
            for(int x=0; x<width; x++){
                int loc = y*width + x;
                double[] p1 = points.get(loc);
                // Look at neighboring pixels
                if(!almostZero(p1)){
                    int[] neighbors = new int[]{y*width + x+1, (y+1)*width + x};
                    for(int loc2 : neighbors){
                        if (loc2>=0 && loc2<points.size() && (x+1)<width){
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
    private boolean inArmRange(ArrayList<GLineSegment2D> armLines, double[] p)
    {
        assert(armWidths.size() <= armLines.size());
        boolean inArm = false;
        for(int i=0; i<armWidths.size(); i++){
            if(armLines.get(i).distanceTo(p) < armWidths.get(i)){
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
        if(eval < 0 && coef[3] > 0)
            return true;
        else if(eval > 0 && coef[3] < 0)
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
}
