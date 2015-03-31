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
        public double fieldSize = 5.0;  // [meters];
        public double fieldRes = 0.1;   // [meters / pixel]

        // attractiveThreshold used for combined method, specifying a distance
        // that, when exceeded, will switch to linear potential from quadratic.
        public AttractivePotential attractivePotential = AttractivePotential.LINEAR;
        public double attractiveWeight = 1.0;
        public double attractiveThreshold = 1.0;
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

        getAttractivePotential(params, pf);

        return pf;
    }

    static private void getAttractivePotential(Params params,
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

    static public void main(String[] args)
    {
        double[] goal = new double[] {4, 2, 0};
        pose_t pose = new pose_t();
        pose.orientation = new double[] {1, 0, 0, 0};
        pose.pos = new double[] {1.5, 1, 0};

        Params params = new Params(new laser_t(), pose, goal);
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


        VisWorld.Buffer vb = vw.getBuffer("potential-field");
        vb.setDrawOrder(-10);
        vb.addBack(pf.getVisObject());
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
