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
import probcog.util.Tree.Node;
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
    int searchDepth = 3;
    int numSamples = 1;

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

        // Preprocessing for heuristics.
        // 1) Build an ordered list of the L2 distances from each tag to the goal.
        // 2) Build our search tree
        Collections.sort(tags, new TagDistanceComparator(goal));
        Tree<Behavior> tree = buildTree(searchDepth, goal);

        // Search tree for node with closest XYT to goal. In-order traversal?
        Node<Behavior> bestNode = tree.root;
        double bestDist = LinAlg.distance(goal, bestNode.data.xyt, 2);
        ArrayList<Node<Behavior> > nodes = tree.inOrderTraversal();
        System.out.println("Tree size: "+ nodes.size());
        for (Node<Behavior> node: nodes) {
            double dist = LinAlg.distance(goal, node.data.xyt, 2);
            if (dist < bestDist) {
                bestDist = dist;
                bestNode = node;
            }
        }

        // Trace back behaviors to reach said node
        while (bestNode.parent != null) {
            behaviors.add(bestNode.data);
            bestNode = bestNode.parent;
        }

        Collections.reverse(behaviors);
        return behaviors;
    }

    private Tree<Behavior> buildTree(int depth, double[] goal)
    {
        double[] xyt = LinAlg.matrixToXYT(robot.getPose());
        Tree<Behavior> tree = new Tree<Behavior>(new Behavior(xyt, null, null));

        // Populate lower levels of tree
        ArrayList<Node<Behavior> > nodes = new ArrayList<Node<Behavior> >();
        nodes.add(tree.root);
        for (int i = 0; i < searchDepth; i++) {
            ArrayList<Node<Behavior> > nextNodes = new ArrayList<Node<Behavior> >();
            for (Node<Behavior> node: nodes) {
                makeChildren(node, goal);
                nextNodes.addAll(node.children);
            }
            nodes = nextNodes;
        }

        return tree;
    }

    private void makeChildren(Node<Behavior> node, double[] goal)
    {
        MonteCarloBot mcb = new MonteCarloBot(sw);

        // Parameters for termination condition
        Set<String> classes = tagdb.getAllClasses();
        HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
        params.put("count", new TypedValue(1)); // We only ever consider counting one object

        // For every combination of termination condition and control law, simulate
        // and evaluate various actions.
        for (FollowWall law: controls) {
            for (String tagClass: classes) {
                int count = 0;
                double[] mean_xyt = new double[3];
                params.put("class", new TypedValue(tagClass));

                for (int i = 0; i < numSamples; i++) {
                    mcb.init(law, new ClassificationCounterTest(params), node.data.xyt);
                    mcb.simulate();
                    if (mcb.success()) {
                        count++;
                        LinAlg.plusEquals(mean_xyt, LinAlg.matrixToXYT(mcb.getPose()));
                    }
                }
                // Check to make sure at least one instance of this plan succeeded
                // XXX This could be more important later. What if we want to actually
                // store a distribution of states in our behavior? For example, we could
                // sample a random XYT from the previous distribution every time when starting
                // or from a distribution estimating the spread of XYTs.
                if (count < 1)
                    continue;
                mean_xyt = LinAlg.scale(mean_xyt, 1.0/count);

                // Should we consider closeness to goal?
                // Store to tree for future consideration
                node.addChild(new Behavior(mean_xyt, law, new ClassificationCounterTest(params)));
            }
        }
    }
}
