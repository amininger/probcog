package probcog.util;

import java.awt.*;
import javax.swing.*;
import java.util.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;
import april.vis.*;

public class BoundingBox
{
    static final double DMAX = Double.POSITIVE_INFINITY;
    static final double DMIN = Double.NEGATIVE_INFINITY;

    // Parameters of the box
    public double[] lenxyz;
    public double[] xyzrpy;

    public BoundingBox()
    {
        lenxyz = new double[3];     // Axis-aligned side lengths for the box
        xyzrpy = new double[6];     // Transforms box centered at origin
    }

    public BoundingBox(double[] lenxyz_, double[] xyzrpy_)
    {
        lenxyz = lenxyz_;
        xyzrpy = xyzrpy_;
    }

    /** Compute the volume of the bounding box */
    public double volume()
    {
        return lenxyz[0]*lenxyz[1]*lenxyz[2];
    }

    // XXX DEBUG
    public void print()
    {
        System.out.printf("[%f x %f x %f]\n", lenxyz[0], lenxyz[1], lenxyz[2]);
        System.out.printf("[%f, %f, %f, %f]\n", xyzrpy[0],
                                                xyzrpy[1],
                                                xyzrpy[2],
                                                xyzrpy[5]);
    }

    public VisObject getVis(Style ... styles)
    {
        VzBox box = new VzBox(lenxyz, styles);
        return new VisChain(LinAlg.xyzrpyToMatrix(xyzrpy), box);
    }

    /** Return the axis-aligned bounding box for a supplied set of points.
     *  The box is defined by 2 opposition corners.
     */
    static BoundingBox getAxisAligned(ArrayList<double[]> points)
    {
        double[] min = new double[] {DMAX, DMAX, DMAX};
        double[] max = new double[] {DMIN, DMIN, DMIN};

        for (double[] p: points) {
            min[0] = Math.min(min[0], p[0]);
            min[1] = Math.min(min[1], p[1]);
            min[2] = Math.min(min[2], p[2]);
            max[0] = Math.max(max[0], p[0]);
            max[1] = Math.max(max[1], p[1]);
            max[2] = Math.max(max[2], p[2]);
        }

        // XXX TEST ME
        BoundingBox bbox = new BoundingBox();
        bbox.lenxyz = LinAlg.subtract(max, min);
        bbox.xyzrpy = LinAlg.resize(LinAlg.add(LinAlg.scale(bbox.lenxyz, .5),
                                               min),
                                    6);
        return bbox;
    }

    /** Assumes in the XY plane. Returns xlen, ylen, xcenter, ycenter, yaw */
    static private double[] getBoundingRectangle(ArrayList<double[]> hull2D)
    {
        double area = Double.POSITIVE_INFINITY;
        double[] best = new double[5];
        for (int i = 0; i < hull2D.size(); i++) {
            double[] p0 = hull2D.get(i);
            double[] p1 = hull2D.get((i+1)%hull2D.size());
            GLine2D lx = new GLine2D(p0, p1);
            GLine2D ly = lx.perpendicularLine();

            double[] min = new double[] {DMAX, DMAX};
            double[] max = new double[] {DMIN, DMIN};
            for (double[] p: hull2D) {
                double xcoord = lx.getLineCoordinate(p);
                double ycoord = ly.getLineCoordinate(p);
                min[0] = Math.min(min[0], xcoord);
                max[0] = Math.max(max[0], xcoord);
                min[1] = Math.min(min[1], ycoord);
                max[1] = Math.max(max[1], ycoord);
            }

            // Project coordinates back into space to find centroid
            GLine2D ly0 = lx.perpendicularLineThrough(lx.getPointOfCoordinate(min[0]));
            GLine2D ly1 = lx.perpendicularLineThrough(lx.getPointOfCoordinate(max[0]));
            GLine2D lx0 = ly.perpendicularLineThrough(ly.getPointOfCoordinate(min[1]));
            GLine2D lx1 = ly.perpendicularLineThrough(ly.getPointOfCoordinate(max[1]));

            double[] min0 = lx0.intersectionWith(ly0);
            double[] max0 = lx1.intersectionWith(ly1);

            double[] lenxy = LinAlg.subtract(max, min);

            if (area < lenxy[0] * lenxy[1])
                continue;

            area = lenxy[0] * lenxy[1];
            best[0] = lenxy[0];
            best[1] = lenxy[1];

            double[] center = LinAlg.add(min0,
                                         LinAlg.scale(LinAlg.subtract(max0, min0), 0.5));

            best[2] = center[0];
            best[3] = center[1];
            best[4] = lx.getTheta();
        }

        return best;
    }

    /** Return the minimal bounding box for a supplied set of points such that
     *  one side of the box is constrained to be parallel to the XY plane.
     *
     *
     */
    static public BoundingBox getMinimalXY(ArrayList<double[]> points)
    {
        // The minimal bounding box constrained to have a box with a face
        // parallel to the XY plane is roughly equivalent to finding the 2D
        // minimal bounding rectangle. This may be found by first computing
        // the convex hull of the the points without considering Z dimensions.
        // Then, the box max be found by considering all boxes that are
        // co-linear with at least one edge of the hull.
        ArrayList<double[]> hull2D = ConvexHull.getHull2D(points);

        // Run through all of the points to find the spread in Z
        double zmin = DMAX;
        double zmax = DMIN;
        for (double[] p: points) {
            zmin = Math.min(zmin, p[2]);
            zmax = Math.max(zmax, p[2]);
        }

        // Consider each edge of the polygon forming the hull of our box in
        // turn and compute the minimal bounding box with an edge aligned with
        // said edge.
        double[] rect = getBoundingRectangle(hull2D);

        BoundingBox minBBox = new BoundingBox(new double[] {rect[0],
                                                            rect[1],
                                                            zmax-zmin},
                                              new double[] {rect[2],
                                                            rect[3],
                                                            zmin + (zmax-zmin)*0.5,
                                                            0,
                                                            0,
                                                            rect[4]});
        return minBBox;
    }

    /** Get an approximate minimal bounding box for a set of points.
     *  Exact minimal bounding boxes can be found in O(n^3) time for
     *  a convex hull bounding the points of size n. However, a bounding
     *  box no more than two times this volume may be found with an
     *  incredibly straightforward algorithm in O(n^2). This is an
     *  implementation of the latter.
     **/
    static public BoundingBox getFastMinimal(ArrayList<double[]> points)
    {
        Polyhedron3D hull = ConvexHull.getHull3D(points);

        // Iterate through the faces of the hull. Find the minimal bounding box
        // with a face flush with a face of the hull.
        ArrayList<double[]> vertices = hull.getVertices();
        ArrayList<int[]> faces = hull.getFaces();
        ArrayList<double[]> normals = hull.getNormals();
        double[] xyzrpy = new double[6];
        double[] lenxyz = new double[3];
        double minVolume = Double.POSITIVE_INFINITY;
        for (int i = 0; i < faces.size(); i++) {
            double[] n = normals.get(i);
            double[] p0 = vertices.get(faces.get(i)[0]);
            double[] p1 = vertices.get(faces.get(i)[1]);
            double[] xaxis = LinAlg.normalize(LinAlg.subtract(p1, p0));
            double[] yaxis = LinAlg.normalize(LinAlg.crossProduct(n, xaxis));

            ArrayList<double[]> projectedPoints = new ArrayList<double[]>();
            for (double[] q: hull.getVertices()) {
                // Find opposite plane face
                double d = Math.abs(LinAlg.dotProduct(LinAlg.subtract(q, p0), n));

                // Project points of hull onto plane, use 2D box method. Define
                // an arbitrary planar coordinate system based on an edge of the
                // face with p0 as the origin.
                double[] xy = new double[2];
                xy[0] = LinAlg.dotProduct(LinAlg.subtract(q, p0), xaxis);
                xy[1] = LinAlg.dotProduct(LinAlg.subtract(q, p0), yaxis);
                projectedPoints.add(xy);




            }

        }

        return new BoundingBox();
    }

    // Utility for random point generation.
    // It would be cool if this took parameters
    static public MultiGaussian makeRandomGaussian3D(Random r)
    {
        double[][] P = new double[3][3];
        double[] u = new double[3];

        // Generate mean
        u[0] = r.nextGaussian()*5;
        u[1] = r.nextGaussian()*5;
        u[2] = r.nextGaussian()*5;

        // Generate diagonals
        P[0][0] = Math.abs(r.nextGaussian()*10);
        P[1][1] = Math.abs(r.nextGaussian()*10);
        P[2][2] = Math.abs(r.nextGaussian()*10);

        System.out.printf("u: [%f, %f, %f]\n", u[0], u[1], u[2]);
        System.out.printf("P: [%f, %f, %f]\n"+
                          "   [%f, %f, %f]\n"+
                          "   [%f, %f, %f]\n", P[0][0], P[0][1], P[0][2],
                                               P[1][0], P[1][1], P[1][2],
                                               P[2][0], P[2][1], P[2][2]);

        return new MultiGaussian(P, u);
    }

    /** Test bounding boxes */
    static public void main(String[] args)
    {
        JFrame jf = new JFrame("Bounding Box Test");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800,600);

        final Random r = new Random(1);
        final VisWorld vw = new VisWorld();
        VzGrid.addGrid(vw);
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        ParameterGUI pg = new ParameterGUI();
        // Covariance stuff
        pg.addIntSlider("num", "Num Points", 4, 10000, 100);
        pg.addButtons("generate", "Generate Box");
        pg.addBoolean("box_align", "Axis-aligned", true);
        pg.addBoolean("box_minz", "Minimal XY", false);
        pg.addBoolean("box_min", "Minimal all", false);
        pg.addListener(new ParameterListener() {
            public void parameterChanged(ParameterGUI pg, String name) {
                if (name.contains("box_")) {
                    pg.sb("box_align", name.equals("box_align"));
                    pg.sb("box_minz", name.equals("box_minz"));
                    pg.sb("box_min", name.equals("box_min"));
                }

                if (name.equals("generate")) {
                    // Make points and box them
                    MultiGaussian mg = makeRandomGaussian3D(r);
                    ArrayList<double[]> points = mg.sampleMany(r,
                                                               pg.gi("num"));

                    // Render points
                    VisWorld.Buffer vb = vw.getBuffer("points");
                    vb.addBack(new VzPoints(new VisVertexData(points),
                                            new VzPoints.Style(Color.red, 2)));
                    vb.swap();

                    // Create the bound box
                    BoundingBox bbox = new BoundingBox();
                    Tic tic = new Tic();
                    if (pg.gb("box_align"))
                        bbox = getAxisAligned(points);
                    else if (pg.gb("box_minz"))
                        bbox = getMinimalXY(points);
                    else if (pg.gb("box_min"))
                        System.err.println("ERR: minimal boxes not yet supported");
                    else
                        System.err.println("ERR: unrecognized bounding box type");
                    //bbox.print();
                    double time = tic.toc();
                    System.out.printf("Computed bounding box in %f s\n", time);

                    vb = vw.getBuffer("bbox");
                    vb.addBack(bbox.getVis(new VzLines.Style(Color.yellow, 1)));
                    vb.swap();

                }
            }
        });
        jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }
}
