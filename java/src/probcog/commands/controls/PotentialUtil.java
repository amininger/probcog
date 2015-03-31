package probcog.commands.controls;

import java.awt.*;
import javax.swing.*;
import java.util.*;

import april.jmat.*;
import april.vis.*;

import probcog.util.*;

import magic2.lcmtypes.*;

/** A utility class for generating and debugging potential functions */
public class PotentialUtil
{
    static public enum AttractivePotential
    {
        LINEAR, QUADRATIC, COMBINED
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
        public double fieldRes = 0.1;   // [meters / pixel]

        // attractiveThreshold used for combined method, specifying a distance
        // that, when exceeded, will switch to linear potential from quadratic.
        public AttractivePotential attractivePotential = AttractivePotential.LINEAR;
        public double attractiveWeight = 1.0;
        public double attractiveThreshold = 1.0;

        //
        public double repulsiveWeight = 100.0;
        public double maxObstacleRange = 3*robotRadius; // XXX
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

        addAttractivePotential(params, pf);
        addRepulsivePotential(params, pf);

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
                        pf.addIndex(x, y, d*kw);
                        break;
                    case QUADRATIC:
                        pf.addIndex(x, y, 0.5*d*d*kw);
                        break;
                    case COMBINED:
                        if (d > kt) {
                            pf.addIndex(x, y, kt*kw*(d-0.5*kt));
                        } else {
                            pf.addIndex(x, y, 0.5*d*d*kw);
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
        double kw = params.repulsiveWeight;
        double kr = params.maxObstacleRange;
        double invKr = 1.0/kr;

        double[] xyt = LinAlg.matrixToXYT(LinAlg.quatPosToMatrix(params.pose.orientation,
                                                                 params.pose.pos));
        double[] invXyt = LinAlg.xytInverse(xyt);

        // Convert laser_t measurements to global coordinates. Ignore ranges
        // that cannot contribute to our potential.
        ArrayList<double[]> points = new ArrayList<double[]>();
        for (int i = 0; i < params.laser.nranges; i++) {
            double r = params.laser.ranges[i];

            // Error value
            if (r < 0)
                continue;
            // Too far away to contribute potential
            if (r > kr)
                continue;
            double t = params.laser.rad0 + i*params.laser.radstep;
            double[] xy = new double[] { r*Math.cos(t), r*Math.sin(t) };
            points.add(LinAlg.transform(xyt, xy));
        }

        // Determine the distance to the closest point
        for (int y = 0; y < pf.getHeight(); y++) {
            for (int x = 0; x < pf.getWidth(); x++) {
                double[] xy = pf.indexToMeters(x, y);
                double d = Double.MAX_VALUE;
                for (double[] pxy: points) {
                    d = Math.min(d, LinAlg.distance(xy, pxy, 2));
                }

                // No potential added
                if (d > kr)
                    continue;
                pf.addIndex(x, y, 0.5*kw*LinAlg.sq(invKr - d));
            }
        }
    }

    static public void main(String[] args)
    {
        double[] goal = new double[] {4, 2, 0};
        pose_t pose = new pose_t();
        pose.orientation = new double[] {1, 0, 0, 0};
        pose.pos = new double[] {1.5, 1, 0};

        // Fake a hallway. Wall on right is 1m away, wall on left is 0.5m
        laser_t laser = new laser_t();
        laser.rad0 = (float)(-Math.PI/2);
        laser.radstep = (float)(Math.toRadians(1));
        laser.nranges = 180;
        laser.ranges = new float[laser.nranges];
        for (int i = 0; i < laser.nranges; i++) {
            double t = laser.rad0 + i*laser.radstep;
            double r = -1;
            if (t < 0) {
                r = -0.5/Math.sin(t);
            } else if (t > 0) {
                r = 1.0/Math.sin(t);
            }
            if (r > 30 || r < 0)
                r = -1;
            laser.ranges[i] = (float)r;
        }

        Params params = new Params(laser, pose, goal);
        params.attractivePotential = AttractivePotential.COMBINED;
        PotentialField pf = PotentialUtil.getPotential(params);

        JFrame jf = new JFrame("Potential test");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        int[] map = new int[] {0xffffff00,
                               0xffff00ff,
                               0xff0000ff,
                               0x0007ffff,
                               0xff2222ff};
        ColorMapper cm = new ColorMapper(map, 0, params.repulsiveWeight/5);

        VisWorld.Buffer vb = vw.getBuffer("potential-field");
        vb.setDrawOrder(-10);
        vb.addBack(pf.getVisObject(cm));
        vb.swap();

        vb = vw.getBuffer("grid");
        vb.addBack(new VzGrid());
        vb.swap();

        vb = vw.getBuffer("robot");
        vb.setDrawOrder(10);
        vb.addBack(new VisChain(LinAlg.quatPosToMatrix(pose.orientation,
                                                       pose.pos),
                                new VzRobot(new VzMesh.Style(Color.green))));
        vb.swap();

        jf.setVisible(true);
    }
}
