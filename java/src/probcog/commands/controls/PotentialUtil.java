package probcog.commands.controls;

import java.awt.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.vis.*;
import april.util.*;

import probcog.util.*;

import magic2.lcmtypes.*;

/** A utility class for generating and debugging potential functions */
public class PotentialUtil
{
    static final boolean DEBUG = false;

    static public enum AttractivePotential
    {
        LINEAR, QUADRATIC, COMBINED
    }

    // XXX Doorway preservation is slow and not great to use, yet.
    static public enum RepulsivePotential
    {
        CLOSEST_POINT, ALL_POINTS, PRESERVE_DOORS
    }

    static public class Params
    {
        public laser_t laser;
        public pose_t pose;
        public double[] goalXYT;

        public Params(laser_t laser, pose_t pose, double[] goalXYT)
        {
            this.laser = laser;
            this.pose = pose;
            this.goalXYT = goalXYT;
        }

        // XXX Other parameters to affect search here. Set to sane default
        // values for normal goto XY operation.

        // Is this value set correctly in config? Reevaluate for new robot.
        public double robotRadius = Util.getConfig().requireDouble("robot.geometry.radius");
        public double fieldSize = 3.0;  // [meters];
        public double fieldRes = 0.05;   // [meters / pixel]

        // attractiveThreshold used for combined method, specifying a distance
        // that, when exceeded, will switch to linear potential from quadratic.
        public AttractivePotential attractivePotential = AttractivePotential.LINEAR;
        public double attractiveWeight = 1.0;
        public double attractiveThreshold = 1.0;

        public RepulsivePotential repulsivePotential = RepulsivePotential.CLOSEST_POINT;
        public double repulsiveWeight = 5.0;
        public double maxObstacleRange = 2*robotRadius;
    }

    static public ArrayList<double[]> getMinPath(double[] rxy_start,
                                                 PotentialField pf)
    {
        ArrayList<double[]> path = new ArrayList<double[]>();

        double[] currXY = LinAlg.copy(rxy_start);
        int iters = 300;
        while (pf.inRange(currXY[0], currXY[1])) {
            path.add(LinAlg.copy(currXY));
            double[] g = getGradient(currXY, pf);
            currXY[0] += g[0]*pf.getMPP()/4;
            currXY[1] += g[1]*pf.getMPP()/4;

            if (iters-- <= 0)
                break;
        }

        return path;
    }

    /** Get a point gradient for a point relative to the robot. No precomputation
     *  of the potential field.
     *
     *  @param rxy      Robot relative coordinate
     *  @param goal     Goal relative to robot
     *  @param params   Potential field params
     */
    static public double[] getGradient(double[] rxy,
                                       double[] goal,
                                       Params params)
    {
        double eps = 0.001;
        double v00 = getRelative(rxy[0], rxy[1], goal, params);
        double v10 = getRelative(rxy[0]+eps, rxy[1], goal, params);
        double v01 = getRelative(rxy[0], rxy[1]+eps, goal, params);
        double v11 = getRelative(rxy[0]+eps, rxy[1]+eps, goal, params);

        if (v00 == Double.MAX_VALUE || v10 == Double.MAX_VALUE ||
            v01 == Double.MAX_VALUE || v11 == Double.MAX_VALUE)
            return new double[2];

        double dx = 0.5*((v10-v00)+(v11-v01))/eps;
        double dy = 0.5*((v01-v00)+(v11-v10))/eps;

        return new double[] {-dx, -dy};
    }

    static public double getRelative(double rx, double ry, double[] goal, Params params)
    {
        double dist = Math.sqrt(LinAlg.sq(rx-goal[0]) + LinAlg.sq(ry-goal[1]));

        double p_att = getAttractivePotential(dist, params);
        double p_rep = getRepulsivePotential(rx, ry, params);

        double p = p_att + p_rep;
        if (Double.isInfinite(p))
            p = Double.MAX_VALUE;

        return p;
    }

    static private double getAttractivePotential(double dist, Params params)
    {
        double kw = params.attractiveWeight;
        double kt = params.attractiveThreshold;

        switch (params.attractivePotential) {
            case LINEAR:
                return dist*kw;
            case QUADRATIC:
                return 0.5*dist*dist*kw;
            case COMBINED:
                if (dist > kt) {
                    return kt*kw*(dist-0.5*kt);
                } else {
                    return 0.5*dist*dist*kw;
                }
            default:
                System.err.println("ERR: Unknown attractive potential");
                return 0;
        }
    }

    static private double getRepulsivePotential(double rx, double ry, Params params)
    {
        double kw = params.repulsiveWeight;
        double kr = params.maxObstacleRange;

        GLineSegment2D line = null;
        if (rx != 0 && ry != 0)
            line = new GLineSegment2D(new double[2], new double[] {rx, ry});

        double[] min = new double[] {Double.MAX_VALUE, Double.MAX_VALUE};
        double acc = 0;
        for (int i = 0; i < params.laser.nranges; i++) {
            double r = params.laser.ranges[i];

            // Error value
            if (r < 0)
                continue;

            double t = params.laser.rad0 + i*params.laser.radstep;
            double[] xy = new double[] { r*Math.cos(t), r*Math.sin(t) };

            double d0 = Math.sqrt(LinAlg.sq(rx-xy[0]) + LinAlg.sq(ry-xy[1]));
            min[0] = Math.min(min[0], d0);
            if (line != null) {
                double d1 = line.distanceTo(xy);
                min[1] = Math.min(min[1], d1); // XXX THIS WON'T WORK
            }
        }

        min[0] = Math.min(min[0], min[1]);
        if (min[0] == 0)
            return Double.MAX_VALUE;

        if (min[0] > kr)
            return 0;
        return 0.5*kw*LinAlg.sq(1.0/min[0] - 1.0/kr);
    }

    /** Get the gradient of a coordinate relative to the robot for the
     *  given potential field. Return as a normalized direction (also relative
     *  to the robot)
     **/
    static public double[] getGradient(double[] rxy,
                                       PotentialField pf)
    {
        double v00 = pf.getRelative(rxy[0], rxy[1]);
        double v10 = pf.getRelative(rxy[0]+pf.getMPP(), rxy[1]);
        double v01 = pf.getRelative(rxy[0], rxy[1]+pf.getMPP());
        double v11 = pf.getRelative(rxy[0]+pf.getMPP(), rxy[1]+pf.getMPP());

        double dx = 0.5*((v10-v00)+(v11-v01));
        double dy = 0.5*((v01-v00)+(v11-v10));

        if (MathUtil.doubleEquals(Math.abs(dx)+Math.abs(dx), 0))
            return new double[2];
        return LinAlg.normalize(new double[] {-dx, -dy});
    }

    /** Given application specific parameters, generate a potential field
     *  locally centered around the robot.
     **/
    static public PotentialField getPotential(Params params)
    {
        double[] robotXYT = LinAlg.matrixToXYT(LinAlg.quatPosToMatrix(params.pose.orientation,
                                                                      params.pose.pos));
        PotentialField pf = new PotentialField(robotXYT,
                                               params.fieldSize,
                                               params.fieldSize,
                                               params.fieldRes);

        Tic tic = new Tic();
        addAttractivePotential(params, pf);
        if (DEBUG)
            System.out.printf("\tattractive: %f [s]\n", tic.toctic());
        addRepulsivePotential(params, pf);
        if (DEBUG)
            System.out.printf("\trepulsive: %f [s]\n", tic.toctic());

        return pf;
    }

    /** Add attrative potential to the system in one of three forms.
     *  1) Linear/conical potential. Directly proportional to distance to goal.
     *  2) Quadratic potential. Square of distance to goal.
     *  3) Combined potential. Starts quadratic, but linear beyond some point.
     **/
    static private void addAttractivePotential(Params params,
                                               PotentialField pf)
    {
        double kw = params.attractiveWeight;
        double kt = params.attractiveThreshold;

        for (int y = 0; y < pf.getHeight(); y++) {
            for (int x = 0; x < pf.getWidth(); x++) {
                double[] xy = pf.indexToMeters(x, y);
                double d = LinAlg.distance(xy, params.goalXYT, 2);

                switch (params.attractivePotential) {
                    case LINEAR:
                        pf.addIndexUnsafe(x, y, d*kw);
                        break;
                    case QUADRATIC:
                        pf.addIndexUnsafe(x, y, 0.5*d*d*kw);
                        break;
                    case COMBINED:
                        if (d > kt) {
                            pf.addIndexUnsafe(x, y, kt*kw*(d-0.5*kt));
                        } else {
                            pf.addIndexUnsafe(x, y, 0.5*d*d*kw);
                        }
                        break;
                    default:
                        System.err.println("ERR: Unknown attractive potential");
                }
            }
        }
    }

    /** Generate repulsive potential based on obstacles observed near the robot.
     *  Derives these range measurements from the provided laser_t in params.
     **/
    static private void addRepulsivePotential(Params params,
                                              PotentialField pf)
    {
        double[] xyt = LinAlg.matrixToXYT(LinAlg.quatPosToMatrix(params.pose.orientation,
                                                                 params.pose.pos));
        double[] invXyt = LinAlg.xytInverse(xyt);
        double sz = params.fieldSize/2;
        double maxRange = params.maxObstacleRange;

        // Convert laser_t measurements to global coordinates. Ignore ranges
        // that cannot contribute to our potential.
        ArrayList<double[]> points = new ArrayList<double[]>();
        for (int i = 0; i < params.laser.nranges; i++) {
            double r = params.laser.ranges[i];

            // Error value
            if (r < 0)
                continue;

            if (r > maxRange + sz)
                continue;

            double t = params.laser.rad0 + i*params.laser.radstep;
            double[] xy = new double[] { r*Math.cos(t), r*Math.sin(t) };
            points.add(LinAlg.transform(xyt, xy));
        }

        switch (params.repulsivePotential) {
            case CLOSEST_POINT:
                repulsiveClosestPoint(pf, points, params);
                break;
            case ALL_POINTS:
                repulsiveAllPoints(pf, points, params);
                break;
            case PRESERVE_DOORS:
            default:
                System.err.println("ERR: Unknown repulsive potential");
        }
    }

    static private void repulsiveClosestPoint(PotentialField pf,
                                              ArrayList<double[]> points,
                                              Params params)
    {
        double kw = params.repulsiveWeight;
        double kr = params.maxObstacleRange;
        double kr2 = kr*kr;
        double invKr = 1.0/kr;

        // Determine the distance to the closest point
        int h = pf.getHeight();
        int w = pf.getWidth();
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                double[] xy = pf.indexToMeters(x, y);
                GLineSegment2D line = null;
                if (xy[0] != 0 && xy[1] != 0)
                    line = new GLineSegment2D(new double[2], xy);

                double d2 = Double.MAX_VALUE;
                double d2_line = Double.MAX_VALUE;
                for (double[] pxy: points) {
                    d2 = Math.min(d2, LinAlg.squaredDistance(xy, pxy, 2));
                    if (line != null)
                        d2_line = Math.min(d2_line, line.squaredDistanceTo(pxy));
                }

                d2 = Math.min(d2, d2_line);

                if (d2 == 0) {
                    pf.addIndexUnsafe(x, y, Double.MAX_VALUE);
                    continue;
                }

                // No potential added
                if (d2 > kr2)
                    continue;
                pf.addIndexUnsafe(x, y, 0.5*kw*LinAlg.sq(1.0/Math.sqrt(d2)-invKr));
            }
        }
    }

    static private void repulsiveAllPoints(PotentialField pf,
                                           ArrayList<double[]> points,
                                           Params params)
    {
        double kw = params.repulsiveWeight;
        double kr = params.maxObstacleRange;
        double kr2 = kr*kr;
        double invKr = 1.0/kr;

        int h = pf.getHeight();
        int w = pf.getWidth();
        for (int y = 0; y < h; y++) {
            for (int x = 0; x < w; x++) {
                double[] xy = pf.indexToMeters(x, y);
                double v = 0;
                int cnt = 0;
                for (double[] pxy: points) {
                    double d2 = LinAlg.squaredDistance(xy, pxy, 2);

                    // No potential added
                    if (d2 > kr2)
                        continue;

                    v += 0.5*LinAlg.sq(1.0/Math.sqrt(d2)-invKr);
                    cnt++;
                }
                if (cnt > 0) {
                    v /= cnt;
                    pf.addIndexUnsafe(x, y, kw*v);
                }
            }
        }
    }

    static public void main(String[] args)
    {
        double[] goal = new double[] {3.0, 0, 0};
        pose_t pose = new pose_t();
        double[] xyt = new double[] {1.5, 1, 0};
        pose.orientation = LinAlg.rollPitchYawToQuat(new double[] {0, 0, xyt[2]});
        pose.pos = new double[] {xyt[0], xyt[1], 0};

        // Fake a hallway. Wall on right is 1m away, wall on left is 0.5m
        laser_t laser = new laser_t();
        laser.rad0 = (float)(-Math.PI);
        laser.radstep = (float)(Math.toRadians(1));
        laser.nranges = 360;
        laser.ranges = new float[laser.nranges];
        double doorOffset = 0.5;
        double doorSize = 0.9;
        for (int i = 0; i < laser.nranges; i++) {
            double t = laser.rad0 + i*laser.radstep;
            double r = -1;
            if (t < 0) {
                r = (-0.5/Math.sin(t));
                if (r*Math.cos(t) > doorOffset && r*Math.cos(t) < doorOffset+doorSize)
                    r = -1;
            } else if (t > 0) {
                r = (1.0/Math.sin(t));
            }
            if (r > 30 || r < 0)
                r = -1;

            laser.ranges[i] = (float)r;
        }

        LCM.getSingleton().publish("TEST_LASER", laser);

        // Construct the potential field
        Params params = new Params(laser, pose, goal);
        params.attractivePotential = AttractivePotential.COMBINED;
        params.fieldSize = 2.0;
        //params.fieldRes = 0.1;
        //params.repulsivePotential = RepulsivePotential.PRESERVE_DOORS;
        //params.maxObstacleRange = 0.1;

        // Wait for keypress
        try {
            System.out.println("Press ENTER to continue:");
            System.in.read();
        } catch (IOException ioex) {}

        Tic tic = new Tic();
        PotentialField pf = PotentialUtil.getPotential(params);
        System.out.printf("Computation completed in %f [s]\n", tic.toc());

        // Evaluate gradients at fixed locations around the robot
        double sz = params.fieldSize/2 - 2*params.fieldRes;
        ArrayList<double[]> rxys = new ArrayList<double[]>();
        for (double y = -sz; y <= sz; y+= 2*params.fieldRes) {
            for (double x = -sz; x <= sz; x+= 2*params.fieldRes) {
                rxys.add(new double[] {x, y});
            }
        }
        ArrayList<double[]> gradients = new ArrayList<double[]>();
        for (double[] rxy: rxys)
            gradients.add(getGradient(rxy, pf));


        JFrame jf = new JFrame("Potential test");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        // Render the field
        int[] map = new int[] {0xffffff00,
                               0xffff00ff,
                               0x0007ffff,
                               0xff0000ff,
                               0xff2222ff};
        double minVal = pf.getMinValue();
        double maxVal = Math.max(5*minVal, params.repulsiveWeight);
        ColorMapper cm = new ColorMapper(map, minVal, maxVal);

        double[][] M = LinAlg.quatPosToMatrix(pose.orientation,
                                              pose.pos);
        VisWorld.Buffer vb = vw.getBuffer("potential-field");
        vb.setDrawOrder(-10);
        vb.addBack(new VisChain(M, pf.getVisObject(cm)));
        vb.swap();

        // Render a grid
        vb = vw.getBuffer("grid");
        vb.addBack(new VzGrid());
        vb.swap();

        // Render a robot
        vb = vw.getBuffer("robot");
        vb.setDrawOrder(10);
        vb.addBack(new VisChain(M, new VzRobot(new VzMesh.Style(Color.green))));
        vb.swap();

        // Render some local potentials
        vb = vw.getBuffer("gradients");
        vb.setDrawOrder(20);
        ArrayList<double[]> bpoints = new ArrayList<double[]>();
        ArrayList<double[]> gpoints = new ArrayList<double[]>();
        for (int i = 0; i < rxys.size(); i++) {
            double[] rxy = rxys.get(i);
            double[] u = gradients.get(i);

            double[] p0 = LinAlg.transform(M, rxy);
            double[] p1 = LinAlg.transform(M, LinAlg.add(LinAlg.scale(u, 1*params.fieldRes), rxy));
            bpoints.add(p0);
            gpoints.add(p0);
            gpoints.add(p1);
        }
        vb.addBack(new VzPoints(new VisVertexData(bpoints),
                                new VzPoints.Style(Color.black, 2)));
        vb.addBack(new VzLines(new VisVertexData(gpoints),
                               VzLines.LINES,
                               new VzLines.Style(Color.gray, 1)));
        vb.swap();

        vb = vw.getBuffer("laser");
        vb.setDrawOrder(100);
        ArrayList<double[]> lpoints = new ArrayList<double[]>();
        for (int i = 0; i < laser.nranges; i++) {
            double r = laser.ranges[i];
            if (r < 0)
                continue;

            double t = laser.rad0 + i*laser.radstep;
            lpoints.add(new double[] {r*Math.cos(t), r*Math.sin(t)});
        }
        vb.addBack(new VisChain(M,
                                new VzPoints(new VisVertexData(lpoints),
                                             new VzPoints.Style(Color.orange, 3))));
        vb.swap();

        //vb = vw.getBuffer("path");
        //vb.setDrawOrder(50);
        //ArrayList<double[]> path = getMinPath(new double[2], pf);
        //vb.addBack(new VisChain(M,
        //                        new VzLines(new VisVertexData(path),
        //                                   VzLines.LINE_STRIP,
        //                                   new VzLines.Style(Color.green, 2))));

        //vb.swap();

        jf.setVisible(true);
    }
}
