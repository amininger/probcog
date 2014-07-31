package probcog.navigation;

import java.io.*;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.classify.*;
import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;
import probcog.sim.*;
import probcog.util.*;
import probcog.vis.*;

// XXX For now, we're assuming we have a simulator representation of the world.
// If we have a map in some other form that the robot uses, we might need to
// have a method for converting this to a sim representation.

/** Using an existing model of the world (like tag locations, etc),
 *  plan the most effective set of behaviors for the robot to follow
 *  from point A to point B.
 **/
public class MonteCarloPlanner
{
    // Search parameters
    int searchDepth = 1;
    int numSamples = 10;

    SimWorld sw;
    SimRobot robot = null;
    ArrayList<SimAprilTag> tags = new ArrayList<SimAprilTag>();
    TagClassifier tagdb;

    ArrayList<FollowWall> controls = new ArrayList<FollowWall>();

    /** Initialize the planner based on a simulated world.
     *  It's assumed that there will only be one robot in this world.
     **/
    public MonteCarloPlanner(SimWorld sw)
    {
        this.sw = sw;
        for (SimObject so: sw.objects) {
            if (so instanceof SimRobot)
                robot = (SimRobot)so;
            else if (so instanceof SimAprilTag)
                tags.add((SimAprilTag)so);
        }

        assert (robot != null);

        // Tag initialization. XXX Know what type each tag is, here?
        try {
            tagdb = new TagClassifier(false);
        } catch (IOException ex) {
            ex.printStackTrace();
            System.exit(1);
        }

        // Initialize planning components
        HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
        params.put("side", new TypedValue((byte)-1));
        controls.add(new FollowWall(params));
        params.put("side", new TypedValue((byte)1));
        controls.add(new FollowWall(params));
    }

    private class TagDistanceComparator implements Comparator<SimAprilTag>
    {
        double[] goal;

        public TagDistanceComparator(double[] goal)
        {
            this.goal = goal;
        }

        public int compare(SimAprilTag a, SimAprilTag b)
        {
            double da = LinAlg.squaredDistance(LinAlg.matrixToXyzrpy(a.getPose()), goal, 2);
            double db = LinAlg.squaredDistance(LinAlg.matrixToXyzrpy(b.getPose()), goal, 2);

            if (da < db)
                return -1;
            else if (da > db)
                return 1;
            return 0;
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof TagDistanceComparator))
                return false;
            TagDistanceComparator tdc = (TagDistanceComparator)o;
            return Arrays.equals(goal, tdc.goal);
        }
    }

    /** Plan a list of behaviors to follow to get within shorter wavefront or
     *  direct drive distance of the goal.
     */
    public ArrayList<Behavior> plan(double[] goal)
    {
        ArrayList<Behavior> behaviors = new ArrayList<Behavior>();
        MonteCarloBot mcb = new MonteCarloBot(sw);

        // Preprocessing for heuristics.
        // 1) Build an ordered list of the L2 distances from each tag to the goal.
        // 2) ...
        Collections.sort(tags, new TagDistanceComparator(goal));

        // XXX Prestore params for only termination condition we use
        // This is troubling, because how do we determine what condition
        // test will get us to stop at a given object? We can really only
        // keep counting up and hope, perhaps targeting specific objects.
        HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
        params.put("count", new TypedValue(1));
        params.put("class", new TypedValue("door"));

        // Search for the set of behaviors that will get the robot closest to the
        // goal from its given position.
        // XXX Worth attempting to append a "drive towards goal" method at end
        // of each step?
        // XXX Would it be necessary/useful to know what each kind of object is
        // before we start our search?
        double[] xyt = LinAlg.matrixToXYT(robot.getPose());
        for (int i = 0; i < searchDepth; i++) {
            double[] best_xyt = xyt;
            FollowWall best_law = null;
            for (FollowWall law: controls) {
                int count = 0;
                double[] mean_xyt = new double[3];
                for (int j = 0; j < numSamples; j++) {
                    ClassificationCounterTest test = new ClassificationCounterTest(params);
                    mcb.init(law, test, xyt);
                    mcb.simulate();
                    if (mcb.success()) {
                        count++;
                        LinAlg.plusEquals(mean_xyt, LinAlg.matrixToXYT(mcb.getPose()));
                    }
                }

                if (count < 1)
                    continue;
                mean_xyt = LinAlg.scale(mean_xyt, 1.0/count);

                // Save best result. Currently just a mean evaluation, but you could
                // imagine evaluating cost based on all the samples and the spread,
                // etc.
                if (LinAlg.squaredDistance(mean_xyt, goal, 2) < LinAlg.squaredDistance(best_xyt, goal, 2)) {
                    best_xyt = mean_xyt;
                    best_law = law;
                }
            }
            // Handle the case in which no useful step is found
            if (best_law == null)
                break;
            behaviors.add(new Behavior(best_law, new ClassificationCounterTest(params)));
        }

        return behaviors;
    }
}
