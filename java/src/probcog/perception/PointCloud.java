package probcog.perception;

import java.awt.*;
import java.util.*;

import april.jmat.*;

import probcog.util.*;

public class PointCloud
{
    private ArrayList<double[]> pts;
    private double[] centroid;
    private BoundingBox bbox;

    /** Initiallize a point cloud with an empty list of points. **/
    public PointCloud()
    {
        pts = new ArrayList<double[]>();
        centroid = null;
    }

    /** PointCloud can be initialized with a single point. **/
    public PointCloud(double[] point)
    {
        pts = new ArrayList<double[]>();
        pts.add(point);
        centroid = null;
    }


    public void addPoint(double[] pt)
    {
        pts.add(pt);
    }

    public void addPoints(ArrayList<double[]> newPts)
    {
        pts.addAll(newPts);
    }

    public ArrayList<double[]> getPoints()
    {
        return pts;
    }

    /** Returns the average location of all the points in the pointcloud. **/
    public double[] getCentroid()
    {
        if(centroid == null)
            centroid = computeCentroid(pts);
        return centroid;
    }

    /** Compute the bounding box of the point cloud */
    public BoundingBox getBoundingBox()
    {
        // Currently implementation uses minimal bounding boxes constrained to
        // have one plane parallel to XY
        return BoundingBox.getMinimalXY(pts);
    }


    /** Compute the average of all the points in the point cloud. Automatically
     *  sets result to centroid. **/
    private double[] computeCentroid(ArrayList<double[]> points)
    {
        double[] center = new double[3];
        if(pts.size() < 1){
            System.err.println("No points in point cloud - can't compute center");
        }
        else{
            for(double[] p : points){
		double[] p3 = new double[]{p[0], p[1], p[2]};
                LinAlg.plusEquals(center, p3);
            }

            for(int i=0; i<center.length; i++){
                center[i] /= (double) points.size();
            }
        }
        return center;
    }

    public int[] getAvgRGB()
    {
        int[] rgb = new int[3];
        for(double[] pt : pts) {
            Color c = new Color((int)pt[3]);
            rgb[0] += c.getRed();
            rgb[1] += c.getGreen();
            rgb[2] += c.getBlue();
        }
        int num = pts.size();
        return new int[]{rgb[0]/num, rgb[1]/num, rgb[2]/num};
    }

    //////////////////////////////////////////////////////
    // Everything below here was originally in BoltUtil

    /** Transform the supplied points such that the area of the box bounding them is
     *  minimized in XY and such that the X coordinate of the box is maximized.
     */
    public ArrayList<double[]> getCanonical()
    {
        double theta = getBBoxTheta(pts);
        ArrayList<double[]> rotated = rotateAtOrigin(pts, theta);
        double[] cxyz = getCentroidXY(rotated);

        double minX = Double.MAX_VALUE;
        double minY = Double.MAX_VALUE;
        double maxX = Double.MIN_VALUE;
        double maxY = Double.MIN_VALUE;
        for (double[] p: rotated) {
            minX = Math.min(minX, p[0]);
            maxX = Math.max(maxX, p[0]);
            minY = Math.min(minY, p[1]);
            maxY = Math.max(maxY, p[1]);
        }

        // Flip the points such that they are in the "canonical" orientation for
        // this shape. This is accomplished by minimizing the distance from the
        // bottom left corner to the centroid
        double lx = cxyz[0] - minX;
        double rx = maxX - cxyz[1];
        double by = cxyz[1] - minY;
        double ty = maxY - cxyz[1];

        double sx = rx < lx ? -1 : 1;
        double sy = ty < by ? -1 : 1;
        ArrayList<double[]> scaled =
             LinAlg.transform(LinAlg.scale(sx, sy, 1), rotated);

        return scaled;
    }

    /** Normalize the points such that min X and min Y == 0 and max X,Y == 1 */
    public ArrayList<double[]> normalize(ArrayList<double[]> points)
    {
        ArrayList<double[]> normalized = new ArrayList<double[]>();
        double minx = Double.MAX_VALUE;
        double miny = Double.MAX_VALUE;
        double maxx = Double.NEGATIVE_INFINITY;
        double maxy = Double.NEGATIVE_INFINITY;
        for (double[] p: points) {
            minx = Math.min(minx, p[0]);
            maxx = Math.max(maxx, p[0]);
            miny = Math.min(miny, p[1]);
            maxy = Math.max(maxy, p[1]);
        }

        // Ranges of values assumed
        double dx = maxx - minx;
        double dy = maxy - miny;

        // dmax is necessary to preserve scaling
        double dmax = Math.max(dx, dy);
        for (double[] p: points) {
            double[] xy = new double[2];
            if (dmax > 0) {
                xy[0] = (p[0]-minx)/dmax;
                xy[1] = (p[1]-miny)/dmax;
            }
            normalized.add(xy);
        }

        return normalized;
    }

    /** Strips all points of their Z coordinate */
    public ArrayList<double[]> flattenXY(ArrayList<double[]> points)
    {
        ArrayList<double[]> flat = new ArrayList<double[]>();

        for (double[] p: points) {
            flat.add(LinAlg.resize(p,2));
        }

        return flat;
    }

    /** Return the rotation around the XY centroid necessary to minimize the
     *  area of the axis-aligned bounding box. Since several such rotations
     *  should exist, pick the orientation with the greatest spread of points
     *  along the X-axis.
     */
    public double getBBoxTheta(ArrayList<double[]> points)
    {
        double[] cxyz = getCentroidXY(points);
        double[][] cXform = LinAlg.translate(cxyz);
        ArrayList<double[]> centered = LinAlg.transform(LinAlg.inverse(cXform), points);

        // Rotate the points incrementally and calculate the area of the bbox.
        // There should be several orientations in which the box is minimized,
        // so choose the one resulting in the longested distribution along the
        // x-axis
        double minArea = Double.MAX_VALUE;
        double theta = 0;
        for (double r = 0; r < 180.0; r += .1)
        {
            ArrayList<double[]> rotated = LinAlg.transform(LinAlg.rotateZ(Math.toRadians(r)),
                                                           centered);

            // Find the area of the bounding box
            double minX = Double.MAX_VALUE;
            double maxX = Double.MIN_VALUE;
            double minY = Double.MAX_VALUE;
            double maxY = Double.MIN_VALUE;
            for (double[] p: rotated) {
                minX = Math.min(minX, p[0]);
                maxX = Math.max(maxX, p[0]);
                minY = Math.min(minY, p[1]);
                maxY = Math.max(maxY, p[1]);
            }

            // Prefer rotations with dominant X axis. We _will_ see an equiv.
            // version for x
            if (maxY - minY > maxX - minX)
                continue;
            double area = (maxX - minX) * (maxY - minY);
            if (area < minArea) {
                minArea = area;
                theta = Math.toRadians(r);
            }
        }

        return theta;
    }

    /** Move the supplied points to the origin and rotate
     *  by the requested angle
     */
    public ArrayList<double[]> rotateAtOrigin(ArrayList<double[]> points, double theta)
    {
        double[] cxyz = getCentroidXY(points);
        double[][] cXform = LinAlg.translate(cxyz);
        ArrayList<double[]> centered = LinAlg.transform(LinAlg.inverse(cXform), points);

        return LinAlg.transform(LinAlg.rotateZ(theta), centered);
    }

    /** Return the XY centroid of the supplied points based
     *  on the upper face of the shape
     */
    public double[] getCentroidXY(ArrayList<double[]> points)
    {
        if (points == null || points.size() < 1)
            return new double[2];

        return computeCentroid(isolateTopFace(points));
    }

    public ArrayList<double[]> isolateTopFace(ArrayList<double[]> points)
    {
        return isolateTopFace(points, 0.005);
    }

    /** Isolates the upper face of our observed shapes.
     *  Discretizes points in the Z dimension based on
     *  the r parameter and then returns all points within
     *  that discretization
     */
    public ArrayList<double[]> isolateTopFace(ArrayList<double[]> points, double r)
    {
        assert (points.size() != 0);
        if (points.get(0).length < 3)
            return points;

        // Find a search range
        double min = Double.MAX_VALUE;
        double max = Double.MIN_VALUE;
        for (double[] p: points) {
            min = Math.min(p[2], min);
            max = Math.max(p[2], max);
        }

        double bestZ = max;
        int bestCnt = 0;

        for (double z = min; z <= max; z+= 0.001) {
            int cnt = 0;
            for (double[] p: points) {
                if (Math.abs(z-p[2]) < r)
                    cnt++;
            }
            if (cnt > bestCnt) {
                bestCnt = cnt;
                bestZ = z;
            }
        }

        ArrayList<double[]> top = new ArrayList<double[]>();
        for (double[] p: points) {
            if (Math.abs(bestZ-p[2]) < r) {
                top.add(p);
            }
        }

        return top;
    }
}
