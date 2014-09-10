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
import probcog.sim.MonteCarloBot.TagRecord;
import probcog.sim.MonteCarloBot.Cluster;
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
    private boolean debug = true;
    VisWorld vw;
    Stopwatch watch = new Stopwatch();

    // Search parameters XXX MOVE TO CONFIG
    boolean iterativeDeepening = Util.getConfig().requireBoolean("monte_carlo.iterative_deepening");
    int searchDepth = Util.getConfig().requireInt("monte_carlo.max_search_depth");
    int numExploreSamples = Util.getConfig().requireInt("monte_carlo.num_exploration_samples");
    int numSamples = Util.getConfig().requireInt("monte_carlo.num_evaluation_samples");

    WavefrontPlanner wfp;
    GridMap gm;
    float[] wf;

    SimWorld sw;
    SimRobot robot = null;
    ArrayList<SimAprilTag> tags = new ArrayList<SimAprilTag>();
    TagClassifier tagdb;

    ArrayList<FollowWall> controls = new ArrayList<FollowWall>();

    private class LawRecordPair
    {
        public double totalDistance;
        public FollowWall law;
        public TagRecord rec;

        public LawRecordPair(double dist, FollowWall law, TagRecord rec)
        {
            this.totalDistance = dist;
            this.law = law;
            this.rec = rec;
        }
    }

    private class LRPComparator implements Comparator<LawRecordPair>
    {
        float[] wf;

        public LRPComparator(float[] wf)
        {
            this.wf = wf;
        }

        public int compare(LawRecordPair a, LawRecordPair b)
        {
            // Downside: now we'd rather reliably go SOMEPLACE rather than the
            // place we want to go.
            // First, bias our search towards repeatability. How consistently
            // can we land our points at effectively the same place?
            Cluster ca = a.rec.getClusters().get(0);
            Cluster cb = b.rec.getClusters().get(0);

            //System.out.printf("%d : %d\n", ca.size(), cb.size());
            //if (ca.size() > cb.size())
            //    return -1;
            //else if (ca.size() < cb.size())
            //    return 1;

            // Next, make sure these points wind up close to the goal. In lots
            // of cases, we expect to have numerous "reliable" options, so it
            // is preferable to choose the ones we perceive as making more
            // forward progress.
            double[] axyt = ca.getMean();
            double[] bxyt = cb.getMean();
            float wa = (float)ca.size()/a.rec.size();
            float wb = (float)cb.size()/b.rec.size();

            float adist = (float)a.totalDistance + (float)a.rec.traveled;
            float bdist = (float)b.totalDistance + (float)b.rec.traveled;

            int ixa = (int) Math.floor((axyt[0]-gm.x0)/gm.metersPerPixel);
            int iya = (int) Math.floor((axyt[1]-gm.y0)/gm.metersPerPixel);
            int ixb = (int) Math.floor((bxyt[0]-gm.x0)/gm.metersPerPixel);
            int iyb = (int) Math.floor((bxyt[1]-gm.y0)/gm.metersPerPixel);
            // assert validity of these indices? They *should* always be inbounds
            float da = (wf[iya*gm.width + ixa]) - 25*wa;
            float db = (wf[iyb*gm.width + ixb]) - 25*wb;
            //float da = (wf[iya*gm.width + ixa] + .5f*adist) - 25*wa;
            //float db = (wf[iyb*gm.width + ixb] + .5f*bdist) - 25*wb;
            //System.out.printf("%f - %f\n", da, db);
            if (da < db)
                return -1;
            else if (da > db)
                return 1;

            return 0;
        }
    }

    // XXX Someday, we won't need to use the sim world, perhaps
    public MonteCarloPlanner(SimWorld sw, GridMap gm)
    {
        this(sw, gm, null);
    }

    /** Initialize the planner based on a simulated world.
     *  It's assumed that there will only be one robot in this world.
     **/
    public MonteCarloPlanner(SimWorld sw, GridMap gm, VisWorld vw)
    {
        this.sw = sw;
        this.gm = gm;
        this.vw = vw;
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
        params.put("distance", new TypedValue((double)0.75));
        controls.add(new FollowWall(params));
        params.put("side", new TypedValue((byte)1));
        params.put("distance", new TypedValue((double)0.75));
        controls.add(new FollowWall(params));

        // Initialize wavefront planner used for rough distance
        this.wfp = new WavefrontPlanner(gm, 0.4);   // XXX
    }

    private class TagDistanceComparator implements Comparator<SimAprilTag>
    {
        //double[] goal;
        float[] wf;

        public TagDistanceComparator(float[] wf)
        {
            //this.goal = goal;
            //this.wf = wfp.getWavefront(null, goal);
            this.wf = wf;
        }

        public int compare(SimAprilTag a, SimAprilTag b)
        {
            //double da = LinAlg.squaredDistance(LinAlg.matrixToXyzrpy(a.getPose()), goal, 2);
            //double db = LinAlg.squaredDistance(LinAlg.matrixToXyzrpy(b.getPose()), goal, 2);
            double[] axy = LinAlg.matrixToXyzrpy(a.getPose());
            double[] bxy = LinAlg.matrixToXyzrpy(b.getPose());

            int ixa = (int) Math.floor((axy[0] - gm.x0) / gm.metersPerPixel);
            int iya = (int) Math.floor((axy[1] - gm.y0) / gm.metersPerPixel);
            int ixb = (int) Math.floor((bxy[0] - gm.x0) / gm.metersPerPixel);
            int iyb = (int) Math.floor((bxy[1] - gm.y0) / gm.metersPerPixel);
            double da = wf[iya * gm.width + ixa];
            double db = wf[iyb * gm.width + ixb];

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
            return (this == tdc);
        }
    }

    /** Plan a list of behaviors to follow to get within shorter wavefront or
     *  direct drive distance of the goal.
     */
    public ArrayList<Behavior> plan(double[] goal)
    {
        watch.start("plan");
        ArrayList<Behavior> behaviors = new ArrayList<Behavior>();

        // Preprocessing for heuristics.
        // 1) Build an ordered list of the L2 distances from each tag to the goal.
        // 2) Build our search tree
        watch.start("preprocessing");
        this.wf = wfp.getWavefront(null, goal);
        Collections.sort(tags, new TagDistanceComparator(wf));
        //Collections.sort(tags, new TagDistanceComparator(goal));
        watch.stop();

        watch.start("DFS");
        Tree<Behavior> tree = dfsSearch(goal);
        watch.stop();

        // Search tree for node with closest XYT to goal. In-order traversal?
        watch.start("Path extraction");
        Node<Behavior> bestNode = tree.root;
        double bestDist = LinAlg.distance(goal, bestNode.data.randomXYT(), 2);  // XXX
        System.out.println("Tree size: "+tree.size());
        ArrayList<Node<Behavior> > nodes = tree.inOrderTraversal();
        for (Node<Behavior> node: nodes) {
            double dist = LinAlg.distance(goal, node.data.randomXYT(), 2);      // XXX
            if (dist < bestDist) {
                bestDist = dist;
                bestNode = node;
            }
        }
        watch.stop();

        // Trace back behaviors to reach said node
        while (bestNode.parent != null) {
            behaviors.add(bestNode.data);
            bestNode = bestNode.parent;
        }

        Collections.reverse(behaviors);
        watch.stop();

        if (debug)
            watch.print();
        return behaviors;
    }

    /** Do a depth first search for the best set of laws to follow */
    private Tree<Behavior> dfsSearch(double[] goal)
    {
        double[] xyt = LinAlg.matrixToXYT(robot.getPose());
        ArrayList<double[]> xyts = new ArrayList<double[]>();
        xyts.add(xyt);
        Tree<Behavior> tree = new Tree<Behavior>(new Behavior(xyts, 0, null, null));

        // iterative deepening search
        int i = 1;
        if (!iterativeDeepening)
            i = searchDepth;

        boolean done = false;
        for (; i <= searchDepth; i++) {
            for (Node<Behavior> leaf: tree.getLeaves()) {
                done = dfsHelper(leaf, goal, leaf.depth, i);
                if (done)
                    break;
            }
            if (done)
                break;
        }
        if (vw != null) {
            VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
            vb.swap(); // Cleanup
        }

        return tree;
    }

    // XXX Depends on fact that tags were already sorted by distance to goal
    private boolean dfsHelper(Node<Behavior> node, double[] goal, int depth, int maxDepth)
    {
        if (debug) {
            System.out.printf("|");
            for (int i = 0; i < depth; i++) {
                System.out.printf("==");
            }
            if (node != null && node.data != null && node.data.law != null && node.data.test != null)
                System.out.printf("%s", node.data.toString());
            else
                System.out.printf("\n");
        }

        if (LinAlg.distance(node.data.randomXYT(), goal, 2) < 3.0) {    // XXX This can't stay
            System.out.printf("XXXXXXXXXXXXXXXXXXXXXXXXXX\n");
            return true;    // Found the goal
        }

        if (depth > maxDepth) {
            System.out.printf("--TOO DEEP--\n");
            return false;
        }

        // XXX How would we incorporate more laws?
        // GOAL: Forward simulate for a fixed period for n iterations and see
        // what possible control laws emerge. We have IDs associated with
        // every obstacle (and could imagine doing so in our real map), so we
        // can claim that we trying to terminate based on sighting obstacle N.
        // It's possible different controls may result in terminations at the
        // same obstacle! We want to keep track of all such possibilities, and
        // we'd like to be able to iterate through them later in order of which
        // ones are closest to our goal.
        ArrayList<LawRecordPair> lrps = new ArrayList<LawRecordPair>();
        MonteCarloBot mcb;
        for (FollowWall law: controls) {
            mcb = new MonteCarloBot(sw);
            for (int i = 0; i < numExploreSamples; i++) {
                mcb.init(law, null, node.data.randomXYT());
                mcb.simulate(); // This will always timeout
            }
            for (TagRecord rec: mcb.tagRecords.values())
                lrps.add(new LawRecordPair(node.data.distanceTraveled, law, rec));
        }

        // Test our record/law pairs based on tag distance to goal
        // and (heuristic warning!) estimated distance traveled. XXX Not implemented yet
        Collections.sort(lrps, new LRPComparator(wf));
        mcb = new MonteCarloBot(sw);
        for (LawRecordPair lrp: lrps) {
            //if (debug) {
            //    System.out.println();
            //    lrp.rec.printStats();
            //}
            double totalDistance = lrp.totalDistance + lrp.rec.traveled;

            HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
            params.put("class", new TypedValue(lrp.rec.tagClass));
            params.put("count", new TypedValue(lrp.rec.count));

            // Try multiple simulations to evaluate the step
            ArrayList<double[]> xyts = new ArrayList<double[]>();
            for (int i = 0; i < numSamples; i++) {
                mcb.init(lrp.law, new ClassificationCounterTest(params), node.data.randomXYT());
                mcb.simulate();
                if (vw != null) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                    vb.setDrawOrder(-500);
                    vb.addBack(mcb.getVisObject());
                    vb.swap();
                }
                if (mcb.success()) {
                    // XXX This pose is not necessarily where we actually are! Would
                    // be preferable to at least calculate it relative to the tag
                    xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                }
            }
            if (xyts.size() < 1)
                continue;
            Node<Behavior> newNode = node.addChild(new Behavior(xyts, totalDistance, lrp.law, new ClassificationCounterTest(params)));

            if (dfsHelper(newNode, goal, depth+1, maxDepth))
                return true;
        }

        return false;
    }
}
