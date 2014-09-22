package probcog.navigation;

import java.io.*;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.classify.*;
import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.sim.*;
import probcog.navigation.Behavior.Cluster;
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

    private class BehaviorSearchComparator implements Comparator<Behavior>
    {
        public int compare(Behavior a, Behavior b)
        {
            Cluster ca = a.getClusters().get(0);
            Cluster cb = b.getClusters().get(0);

            float wa = (float)ca.size()/(float)numExploreSamples;
            float wb = (float)cb.size()/(float)numExploreSamples;

            // Bias our searches to get us close to the goal fast
            float adist = (float)a.getMeanDistTraveled();
            float bdist = (float)b.getMeanDistTraveled();

            float awf = (float)a.getMeanDistToGoal(gm, wf);
            float bwf = (float)b.getMeanDistToGoal(gm, wf);

            // assert validity of these indices? They *should* always be inbounds
            //float da = (wf[iya*gm.width + ixa] + adist);// - 30.0f*wa;
            //float db = (wf[iyb*gm.width + ixb] + bdist);// - 30.0f*wb;
            float da = awf;
            float db = bwf;
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
        params.put("distance", new TypedValue((double)0.85));
        controls.add(new FollowWall(params));
        params.put("side", new TypedValue((byte)1));
        controls.add(new FollowWall(params));

        // Initialize wavefront planner used for rough distance
        this.wfp = new WavefrontPlanner(gm, 0.5);   // XXX
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

    public ArrayList<Behavior> plan(ArrayList<double[]> starts,
                                    double[] goal)
    {
        return plan(starts, goal, null);
    }

    /** Plan a list of behaviors to follow to get within shorter wavefront or
     *  direct drive distance of the goal.
     */
    public ArrayList<Behavior> plan(ArrayList<double[]> starts,
                                    double[] goal,
                                    SimAprilTag optionalTargetTag)
    {
        goalTag = optionalTargetTag;

        watch.start("plan");
        ArrayList<Behavior> behaviors = new ArrayList<Behavior>();

        // Preprocessing for heuristics.
        // 1) Build an ordered list of the L2 distances from each tag to the goal.
        // 2) Build our search tree
        watch.start("preprocessing");
        this.wf = wfp.getWavefront(null, goal);
        Collections.sort(tags, new TagDistanceComparator(wf));
        watch.stop();

        watch.start("DFS");
        Node<Behavior> soln = dfsSearch(starts, goal);
        watch.stop();

        // Trace back behaviors to reach said node
        while (soln != null && soln.parent != null) {
            behaviors.add(soln.data);
            soln = soln.parent;
        }

        Collections.reverse(behaviors);

        if (debug)
            watch.print();
        return behaviors;
    }

    /** Do a depth first search for the best set of laws to follow */
    private SimAprilTag goalTag;
    private Node<Behavior> soln;
    private double solnScore;
    private Node<Behavior> dfsSearch(ArrayList<double[]> starts, double[] goal)
    {
        //double[] xyt = LinAlg.matrixToXYT(robot.getPose());
        //Tree<Behavior> tree = new Tree<Behavior>(new Behavior(xyt, 0.0, null, null));
        ArrayList<Double> dists = new ArrayList<Double>();
        for (double[] s: starts)
            dists.add(0.0);

        Tree<Behavior> tree = new Tree<Behavior>(new Behavior(starts,
                                                              dists,
                                                              null,
                                                              null));
        // iterative deepening search, if called for. We're trying
        // to avoid this, for now.
        int i = 1;
        if (!iterativeDeepening)
            i = searchDepth;

        // Reset state from last search
        soln = null;
        solnScore = Double.MAX_VALUE;
        for (; i <= searchDepth; i++) {
            for (Node<Behavior> leaf: tree.getLeaves()) {
                dfsHelper(leaf, goal, leaf.depth, i);
                if (soln != null)
                    break;
            }
            if (soln != null)
                break;
        }
        if (vw != null) {
            VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
            vb.swap(); // Cleanup
        }

        return soln;
    }

    private void dfsHelper(Node<Behavior> node, double[] goal, int depth, int maxDepth)
    {
        MonteCarloBot mcb;
        if (debug) {
            System.out.printf("|");
            for (int i = 0; i < depth; i++) {
                System.out.printf("==");
            }
            /*if (node != null && node.data != null && node.data.law != null && node.data.test != null)
                System.out.printf("%s", node.data.toString());
            else*/
                System.out.printf("\n");
        }

        // Prune out solutions that are worse than our best so far.
        double nodeScore = node.data.getBestScore(gm, wf, numSamples, depth);
        if (nodeScore > solnScore) {
            System.out.printf("--PRUNED: [%f < %f] --\n", solnScore, nodeScore);
            return;
        }

        // This must have a better score than we currently have, because we got past
        // the pruning filter. Thus, this is a better solution.
        double pct = node.data.getPctNearGoal(gm, wf, numSamples);
        double ARRIVAL_RATE_THRESH = 0.1;
        if (pct >= ARRIVAL_RATE_THRESH) {
            System.out.printf("XXXXXXXXXXXXXXXXXXXXXXXXXX\n");

            // Time to try out hard-coded last step of attacking the goal tag
            if (goalTag != null) {
                HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
                params.put("id", new TypedValue(goalTag.getID()));
                DriveTowardsTag dtt = new DriveTowardsTag(params);
                params.put("distance", new TypedValue(0.5));

                ArrayList<double[]> xyts = new ArrayList<double[]>();
                ArrayList<Double> distances = new ArrayList<Double>();

                mcb = new MonteCarloBot(sw);
                for (int i = 0; i < numSamples; i++) {
                    Behavior.XYTPair pair = node.data.randomXYT();
                    mcb.init(dtt, new NearTag(params), pair.xyt, pair.dist);
                    mcb.simulate(10.0);
                    if (vw != null) {
                        VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                        vb.setDrawOrder(-500);
                        vb.addBack(mcb.getVisObject());
                        vb.swap();
                    }
                    if (mcb.success()) {
                        // Find where we are and how much we've driven to get there
                        xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                        distances.add(mcb.getTrajectoryLength());
                    }
                }

                Node<Behavior> newNode = node.addChild(new Behavior(xyts, distances, dtt, new NearTag(params)));
                node = newNode;
            }

            soln = node;
            solnScore = soln.data.getBestScore(gm, wf, numSamples, depth);
            return;
        }

        // Don't search beyond our max depth.
        if (depth >= maxDepth) {
            System.out.printf("--TOO DEEP--\n");
            return;
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
        ArrayList<Behavior> recs = new ArrayList<Behavior>();
        for (FollowWall law: controls) {
            mcb = new MonteCarloBot(sw);
            for (int i = 0; i < numExploreSamples; i++) {
                Behavior.XYTPair pair = node.data.randomXYT();
                mcb.init(law, null, pair.xyt, pair.dist);
                mcb.simulate(); // This will always timeout
            }
            for (Behavior rec: mcb.tagRecords.values())
                recs.add(rec);
        }

        // Test our record/law pairs based on tag distance to goal
        // and (heuristic warning!) estimated distance traveled.
        Collections.sort(recs, new BehaviorSearchComparator());
        mcb = new MonteCarloBot(sw);
        for (Behavior rec: recs) {
            // Prune out solutions that are worse than our best so far.
            // This is an early attempt at pruning to avoid spending more time
            // simulating many branches.
            // This is probably NOT a great thing to do, since we don't have
            // all that many samples.
            double recScore = rec.getBestScore(gm, wf, numExploreSamples, depth+1);
            if (solnScore < recScore) {
                System.out.printf("--EARLY PRUNED: [%f < %f]--\n", solnScore, recScore);
                continue;
            }


            // Try multiple simulations to evaluate the step
            ArrayList<double[]> xyts = new ArrayList<double[]>();
            ArrayList<Double> distances = new ArrayList<Double>();
            for (int i = 0; i < numSamples; i++) {
                Behavior.XYTPair pair = node.data.randomXYT();
                mcb.init(rec.law, ((ClassificationCounterTest)rec.test).clone(), pair.xyt, pair.dist);
                mcb.simulate();
                if (vw != null) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                    vb.setDrawOrder(-500);
                    vb.addBack(mcb.getVisObject());
                    vb.swap();
                }
                if (mcb.success()) {
                    // Find where we are and how much we've driven to get there
                    xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                    distances.add(mcb.getTrajectoryLength());
                }
            }
            if (xyts.size() < 1)
                continue;
            Node<Behavior> newNode = node.addChild(new Behavior(xyts, distances, rec.law, ((ClassificationCounterTest)rec.test).clone()));

            dfsHelper(newNode, goal, depth+1, maxDepth);
        }

        return;
    }
}
