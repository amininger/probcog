package probcog.navigation;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.navigation.*;
import probcog.sim.*;

import magic2.lcmtypes.*;

/** Run on robot to do planning based on a map (possible supplied at
 *  runtime, otherwise, preconstructed) in control law space.
 **/
public class RobotNavigator implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();
    String tagChannel;

    SimWorld world;
    GridMap gm; // Eventually populated by SLAM, for now, populated by sim?

    public RobotNavigator(GetOpt gopt) throws IOException
    {
        if (gopt.wasSpecified("world")) {
            Config config = new Config();
            world = new SimWorld(gopt.getString("world"), config);

            createGridMap();
        } else {
            System.err.println("ERR: Currently only support sim mode");
            assert (false);
        }

        tagChannel = gopt.getString("tag-channel");
        lcm.subscribe(tagChannel, this);
    }

    private void createGridMap()
    {
        if (world.objects.size() < 1)
            return;

        // Set dimensions
        double max[] = {-Double.MAX_VALUE, - Double.MAX_VALUE,- Double.MAX_VALUE};
        double min[] = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        for (SimObject so : world.objects) {
            double T[][] = so.getPose();
            april.sim.Shape s = so.getShape();
            if (s instanceof BoxShape) {
                BoxShape bs = (BoxShape) s;

                ArrayList<double[]> vertices = bs.getVertices();

                for (double vertex[] : vertices) {
                    double global_v[] = LinAlg.transform(T, vertex);

                    for (int l = 0; l < 3; l++) {
                        max[l] = Math.max(global_v[l],max[l]);
                        min[l] = Math.min(global_v[l],min[l]);
                    }
                }

            } else if (s instanceof SphereShape){
                SphereShape ss = (SphereShape) s;
                double r = ss.getRadius();
                for (int l = 0; l < 3; l++) {
                    max[l] = Math.max(T[l][3] + r, max[l]);
                    min[l] = Math.min(T[l][3] - r, min[l]);
                }

            } else {
                for (int l = 0; l < 3; l++) {
                    max[l] = Math.max(T[l][3],max[l]);
                    min[l] = Math.min(T[l][3],min[l]);
                }
                System.out.println("WRN: Unsupported shape type: "+s.getClass().getName());
            }
        }

        double MPP = 0.1;
        double[] down = new double[] {0, 0, -1};
        gm = GridMap.makeMeters(min[0], min[1], max[0]-min[0], max[1]-min[1], MPP, 255);

        // XXX There's probably a faster way to do this, but this was easy and it's
        // a one-time thing
        for (double y = min[1]; y < max[1]; y+=.5*MPP) {
            for (double x = min[0]; x < max[0]; x+=.5*MPP) {
                for (SimObject obj: world.objects) {
                    if (!(obj instanceof SimBox))
                        continue;
                    if (Collisions.collisionDistance(new double[] {x, y, 100}, down, obj.getShape(), obj.getPose()) < Double.MAX_VALUE) {
                        gm.setValue(x, y, (byte)0);
                    }
                }
            }
        }
    }

    /** Given a goal in global XYT, compute a plan to execute in control law
     *  space.
     *
     *  Note that the robot will localize itself based on tags, teleporting
     *  around the sim world as needed.
     **/
    synchronized public void computePlan(double[] goalXYT)
    {
        MonteCarloPlanner mcp = new MonteCarloPlanner(world, gm, null);

        // For now, base starting position on wherever the robot was
        // last placed in the world and the closest tag to our target
        // goal. Maybe someday we will have better localization and the ability
        // to GOTO XY and be a little smarter about final goals/termination
        // conditions.
        SimRobot robot = getRobot();
        assert (robot != null);
        SimAprilTag tag = getClosestTag(goalXYT);
        assert (tag != null);

        ArrayList<double[]> starts = new ArrayList<double[]>();
        starts.add(LinAlg.matrixToXYT(robot.getPose()));

        ArrayList<Behavior> behaviors = mcp.plan(starts, goalXYT, tag);
    }

    // === LCM ===
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            // Handle LCM messages as appropriate. For example, we will try to
            // localize off tag detections relative to actual positions in the
            // real world.
            if (channel.equals(tagChannel)) {
                tag_detection_list_t tdl = new tag_detection_list_t(ins);
                handleTags(tdl);
            }
        } catch (IOException ex) {
            System.err.println("ERR: Could not handle message on channel - "+channel);
            ex.printStackTrace();
        }
    }

    synchronized private void handleTags(tag_detection_list_t tdl)
    {
        // Correct robot pose based on tags. In the presence of multiple
        // tags from the sim world, use the average position.
        // XXX We need to eliminate tags that might also be robots!

        // XXX TODO
    }

    // === UTILITY ===
    private SimRobot getRobot()
    {
        assert (world != null);
        for (SimObject so: world.objects) {
            if (!(so instanceof SimRobot))
                continue;
            return (SimRobot)so;
        }

        return null;
    }

    private SimAprilTag getClosestTag(double[] xy)
    {
        SimAprilTag tag = null;
        double dist = Double.MAX_VALUE;
        for (SimObject so: world.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            double d = LinAlg.distance(xy, LinAlg.matrixToXYT(so.getPose()), 2);
            if (d < dist) {
                dist = d;
                tag = (SimAprilTag)so;
            }
        }

        return tag;
    }

    // =========================================================================
    static public void main(String[] args)
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help screen");
        gopt.addString('w', "world", null, "(Someday Optional) sim world file");
        gopt.addString('\0', "tag-channel", "TAG_DETECTIONS", "Tag detection channel");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            System.err.printf("Usage: %s [options]\n", args[0]);
            gopt.doHelp();
            System.exit(1);
        }

        try {
            new RobotNavigator(gopt);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
