package probcog.navigation;

import java.awt.*;
import java.awt.image.*;
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
    private boolean debug = false;
    VisWorld vw;
    Stopwatch watch = new Stopwatch();

    // Search parameters XXX MOVE TO CONFIG. Get rid of ROBOT_CONFIG bit?
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

    private class GreedySearchNode
    {
        public Node<Behavior> node;        // The actual search node
        private ArrayList<Behavior> futureChildren = null;
        private int nextChildIdx = -1;

        public GreedySearchNode(Node<Behavior> node)
        {
            this.node = node;
        }

        public void addSortedChildren(ArrayList<Behavior> children,
                                      Comparator<Behavior> childComparator)
        {
            Collections.sort(children, childComparator);
            this.futureChildren = children;
            nextChildIdx = 0;
        }

        public boolean madeChildren()
        {
            return futureChildren != null;
        }

        public boolean hasNextChild()
        {
            assert (futureChildren != null);
            return nextChildIdx < futureChildren.size();
        }

        public Behavior getNextChild()
        {
            assert (hasNextChild());
            return futureChildren.get(nextChildIdx++);
        }
    }

    private class SpanningTreeComparator implements Comparator<GreedySearchNode>
    {
        public int compare(GreedySearchNode a, GreedySearchNode b)
        {
            // XXX Use of "prob" limits us, like DART, to only picking
            // trajectories where things will happen as expected...
            double ascore = a.node.data.getScoreSoFar(a.node.data.prob);
            double bscore = b.node.data.getScoreSoFar(b.node.data.prob);

            if (ascore < bscore)
                return -1;
            else if (ascore > bscore)
                return 1;
            return 0;
        }
    }

    private class GSNComparator implements Comparator<GreedySearchNode>
    {
        double EPSILON = 0.2;

        public int compare(GreedySearchNode a, GreedySearchNode b)
        {
            double ascore = a.node.data.getBestScore(gm, wf, a.node.data.prob, 0);
            double bscore = b.node.data.getBestScore(gm, wf, b.node.data.prob, 0);

            double adist = a.node.data.getMeanDistToGoal(gm, wf);
            double bdist = b.node.data.getMeanDistToGoal(gm, wf);

            double diff = Math.abs(ascore - bscore);
            double maxDepth = Math.max(a.node.depth, b.node.depth);

            // If scores are sufficiently close, treat as equivalent and
            // order in favor of actual distance traveled instead of
            // estimated
            if (diff < EPSILON*maxDepth) {
                if (adist < bdist) {
                    return -1;
                } else if (adist > bdist) {
                    return 1;
                }
                //if (a.node.depth > b.node.depth)
                //    return -1;
                //else if (a.node.depth < b.node.depth)
                //    return 1;
            } else if (ascore < bscore) {
                return -1;
            } else if (ascore > bscore) {
                return 1;
            }
            return 0;
        }
    }

    // Used to order children for expansion in best-first search.
    private class GreedyChildComparator implements Comparator<Behavior>
    {
        public int compare(Behavior a, Behavior b)
        {
            return closerToGoal(a, b);
        }

        // Since we haven't done any simulation yet, assumes perfect execution.
        // Chooses to expand nodes geting us closer to the goal first?
        private int closerToGoal(Behavior a, Behavior b)
        {
            float adist = (float)a.getMeanDistTraveled();
            float bdist = (float)b.getMeanDistTraveled();

            float awf = (float)a.getMeanDistToGoal(gm, wf);
            float bwf = (float)b.getMeanDistToGoal(gm, wf);

            // Prefer facing towards the wavefront slightly
            float abonus = (float)a.getMeanDirectionalBonus(gm, wf);
            float bbonus = (float)b.getMeanDirectionalBonus(gm, wf);

            // Only look at estimated distance remaining
            double da = awf + abonus;   // Multiplier on bonuses?
            double db = bwf + bbonus;

            if (da < db)
                return -1;
            else if (da > db)
                return 1;
            return 0;
        }
    }

    // We want to minimize score, so return proper values. This is what is
    // used in the standard heap search (best-first in AAMAS paper)
    private class BNComparator implements Comparator<Node<Behavior> >
    {
        public int compare(Node<Behavior> a, Node<Behavior> b) {
            // XXX Depth unused
            double ascore = a.data.getBestScore(gm, wf, a.data.prob, 0);
            double bscore = b.data.getBestScore(gm, wf, b.data.prob, 0);

            if (ascore < bscore)
                return -1;
            else if (ascore > bscore)
                return 1;
            return 0;
        }

        public boolean equals(Object o)
        {
            return (o == this);
        }
    }


    // Used by DFS to evaluate which node to pursue next. This happens AFTER
    // the initial simulation!
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

            // Prefer facing towards the wavefront
            float abonus = (float)a.getMeanDirectionalBonus(gm, wf);
            float bbonus = (float)b.getMeanDirectionalBonus(gm, wf);

            // assert validity of these indices? They *should* always be inbounds
            //float da = (wf[iya*gm.width + ixa] + adist);// - 30.0f*wa;
            //float db = (wf[iyb*gm.width + ixb] + bdist);// - 30.0f*wb;
            double da = (awf + 5*abonus);
            double db = (bwf + 5*bbonus);
            if (da < db)
                return -1;
            else if (da > db)
                return 1;

            return 0;
        }
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

    // ========================================================================
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
    // ========================================================================

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
        watch = new Stopwatch();
        goalTag = optionalTargetTag;

        watch.start("plan");
        ArrayList<Behavior> behaviors = new ArrayList<Behavior>();

        // Preprocessing for heuristics.
        // 1) Build an ordered list of the L2 distances from each tag to the goal.
        // 2) Build our search tree
        watch.start("preprocessing");
        this.wf = wfp.getWavefront(null, goal);

        if (debug) {
            // Render the wavefront
            BufferedImage im = new BufferedImage(gm.width, gm.height, BufferedImage.TYPE_BYTE_GRAY);
            byte[] buf = ((DataBufferByte) (im.getRaster().getDataBuffer())).getData();
            for (int i = 0; i < wf.length; i++) {
                byte v = (byte)255;
                if (wf[i] == Float.MAX_VALUE)
                    v = (byte)0;
                else if (wf[i] > 0)
                    v = (byte)(wf[i]*255/100.0);
                buf[i] = v;
            }

            if (vw != null) {
                VisWorld.Buffer vb = vw.getBuffer("debug-wavefront");
                vb.setDrawOrder(-1001);
                vb.addBack(new VisChain(LinAlg.translate(gm.x0, gm.y0),
                                        LinAlg.scale(gm.metersPerPixel),
                                        new VzImage(new VisTexture(im,
                                                                   VisTexture.NO_MIN_FILTER |
                                                                   VisTexture.NO_MAG_FILTER))));
                vb.swap();
            }
        }
        Collections.sort(tags, new TagDistanceComparator(wf));
        watch.stop();

        watch.start("search");
        //Node<Behavior> soln = dfsSearch(starts, goal);
        //Node<Behavior> soln = heapSearch(starts, goal);
        Node<Behavior> soln = hybridSearch(starts, goal);
        watch.stop();

        System.out.println("NODES EXPANDED: "+nodesExpanded);

        // Trace back behaviors to reach said node
        while (soln != null && soln.parent != null) {
            behaviors.add(soln.data);
            soln = soln.parent;
        }

        Collections.reverse(behaviors);
        Behavior last = behaviors.get(behaviors.size()-1);
        System.out.printf("FINAL SCORE: %f\n", last.getBestScore(gm, wf, last.prob, 0));

        watch.stop();
        watch.print();
        if (debug) {
            if (vw != null) {
                VisWorld.Buffer vb = vw.getBuffer("debug-wavefront");
                vb.swap();
            }
        }
        return behaviors;
    }

    /** Add a DriveTowardsTag step when appropriate */
    private Node<Behavior> finishPlan(Node<Behavior> node,
                                      SimAprilTag goalTag,
                                      double pct)
    {
        HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
        params.put("id", new TypedValue(goalTag.getID()));
        DriveTowardsTag dtt = new DriveTowardsTag(params);
        params.put("distance", new TypedValue(0.5));

        ArrayList<double[]> xyts = new ArrayList<double[]>();
        ArrayList<Double> distances = new ArrayList<Double>();

        MonteCarloBot mcb = new MonteCarloBot(sw);
        for (int i = 0; i < numSamples; i++) {
            Behavior.XYTPair pair = node.data.randomXYT();
            mcb.init(dtt, new NearTag(params), pair.xyt, pair.dist);
            mcb.simulate(10.0); // XXX Another magic number
            if (vw != null) {
                VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                vb.setDrawOrder(-500);
                vb.addBack(mcb.getVisObject());
                vb.swap();
            }
            // Find where we are and how much we've driven to get there
            xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
            distances.add(mcb.getTrajectoryLength());
        }

        Behavior b = new Behavior(xyts, distances, dtt, new NearTag(params));
        // XXX Is this only possible because of our model?
        if (node.data != null)
            b.prob = Math.min(pct, node.data.prob);
        Node<Behavior> newNode = node.addChild(b);
        System.out.printf("\tSCORE: %f\n", b.getBestScore(gm, wf, b.prob, 0));

        return newNode;
    }

    // XXX Notable points of failure could be...
    // 1) The fact that the output of this step is HIGHLY dependent on basin
    //    of convergence for our control law. Why should we believe our sim
    //    and our real robot wind up in a close enough space for this to be
    //    true?
    // 2) How does the forward simulation time affect our search time and
    //    the optimality of our route? Too short and no good route may be
    //    feasible. Too long and we just might spend inordinate amounts of time
    //    simulating. Cycle detection necessary? The state space can make this
    //    challenging. Would prefer to be able to determine basins in advance.
    /** Forward simulate for the maximum allowed time (set in config) to see
     *  what possible actions we might take. Note that this step assumes
     *  perfect sensing of landmarks.
     **/
    private ArrayList<Behavior> generateChildren(Node<Behavior> node) {
        ArrayList<Behavior> recs = new ArrayList<Behavior>();
        for (FollowWall law: controls) {
            MonteCarloBot mcb = new MonteCarloBot(sw);
            for (int i = 0; i < numExploreSamples; i++) {
                // XXX XXX XXX XXX XXX XXX XXX XXX XXX XXX XXX XXX XXX XXX XXX
                // XXX This aspect could backfire. Random XYT with 1 sample?!
                Behavior.XYTPair pair = node.data.randomXYT();
                mcb.init(law, null, pair.xyt, pair.dist);
                mcb.simulate(); // This will always timeout
            }
            for (Behavior rec: mcb.tagRecords.values()) {
                recs.add(rec);
            }
        }

        // Special case: we also consider turning in place at the beggining
        // phase of planning.
        if (node.depth == 0) {
            double[] yaws = new double[] {Math.PI/2, Math.PI, 3*Math.PI/2};
            for (double yaw: yaws) {
                HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
                params.put("direction", new TypedValue((byte)1));  // Left turn in place
                params.put("yaw", new TypedValue(yaw));
                params.put("no-lcm", new TypedValue(0));
                HashMap<String, TypedValue> params2 = new HashMap<String, TypedValue>();
                params2.put("yaw", new TypedValue(yaw));
                params2.put("no-lcm", new TypedValue(0));

                ArrayList<double[]> xyts = new ArrayList<double[]>();
                ArrayList<Double> dists = new ArrayList<Double>();
                MonteCarloBot mcb = new MonteCarloBot(sw);
                for (int i = 0; i < numExploreSamples; i++) {
                    Turn turn = new Turn(params);
                    RotationTest rt = new RotationTest(params2);
                    Behavior.XYTPair pair = node.data.randomXYT();
                    mcb.init(turn, rt, pair.xyt, pair.dist);
                    mcb.simulate();
                    // Only consider plans that actually "work"
                    if (mcb.success()) {
                        xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                        dists.add(mcb.getTrajectoryLength());
                    }
                }

                recs.add(new Behavior(xyts,
                                      dists,
                                      new Turn(params),
                                      new RotationTest(params2)));
            }
        }

        return recs;
    }

    /** Simulate multiple executions of the given behavior and build a
     *  distribution of end states. Return a new instance of the behavior
     *  containing simulation statistics
     **/
    private Behavior simulateBehavior(Behavior b, Node<Behavior> node) {
        MonteCarloBot mcb = new MonteCarloBot(sw);
        // Try multiple simulations to evaluate each possible step
        ArrayList<double[]> xyts = new ArrayList<double[]>();
        ArrayList<Double> distances = new ArrayList<Double>();
        for (int i = 0; i < numSamples; i++) {
            Behavior.XYTPair pair = node.data.randomXYT();
            mcb.init(b.law, (ConditionTest)b.test.copyCondition(), pair.xyt, pair.dist);
            //mcb.simulate(70.0); // XXX How to choose?
            mcb.simulate(300.0); // XXX
            if (vw != null) {
                VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                vb.setDrawOrder(-500);
                vb.addBack(mcb.getVisObject());
                vb.swap();
            }
            // Why do we only count success? Because
            if (mcb.success()) {
                // Find where we are and how much we've driven to get there
                xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                distances.add(mcb.getTrajectoryLength());
            }
        }
        if (xyts.size() < 1)
            return null;    // Complete and utter failure of execution
        Behavior behavior = new Behavior(xyts,
                                         distances,
                                         b.law,
                                         b.test.copyCondition());
        if (node.data != null)
            behavior.prob = node.data.prob;
        behavior.prob *= b.prob;
        behavior.tagID = node.data.tagID;   // COPY OVER TAG ID! Awful bookkeeping
        System.out.printf("\t%s SCORE: %f\n", behavior.law.toString(), behavior.getBestScore(gm, wf, behavior.prob, 0));

        return behavior;
    }

    // ========================================================================
    // === Spanning Tree Building =============================================
    // ========================================================================
    /** Get a tag from the world */
    private SimAprilTag getTag(int tagID)
    {
        for (SimObject so: sw.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            SimAprilTag tag = (SimAprilTag)so;
            if (tag.getID() == tagID)
                return tag;
        }

        return null;
    }

    private ArrayList<SimAprilTag> getOtherTags(int tagID)
    {
        ArrayList<SimAprilTag> otherTags = new ArrayList<SimAprilTag>();
        for (SimObject so: sw.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            SimAprilTag tag = (SimAprilTag)so;
            if (tag.getID() != tagID)
                otherTags.add(tag);
        }

        return otherTags;
    }

    /** Build a spanning tree describing trajectories from the given to all
     *  others.
     **/
    public Tree<Behavior> buildSpanningTree(int tagID)
    {
        SimAprilTag tag = getTag(tagID);
        ArrayList<SimAprilTag> otherTags = getOtherTags(tagID);
        assert (tag != null);

        double[] xyt_0 = LinAlg.matrixToXYT(tag.getPose());
        xyt_0[2] = 0;

        // Initialize the tree as if we were facing down the X-axis at the
        // tag location. It is up to our search to escape efficiently.
        // Is there a place for turn-in-place? I expect yes, and that we'll
        // want that.
        Tree<Behavior> tree = new Tree<Behavior>(new Behavior(xyt_0, 0, null, null));

        // Find the best routes to all other locations simultaneously. Reward
        // a combination of distance traveled and arrival rate. Basically,
        // normal scoring without wavefront predictions
        PriorityQueue<GreedySearchNode> heap =
            new PriorityQueue<GreedySearchNode>(10, new SpanningTreeComparator());
        heap.add(new GreedySearchNode(tree.root));

        Set<SimAprilTag> visitedTags = new HashSet<SimAprilTag>();
        visitedTags.add(tag);

        while (heap.size() > 0) {
            GreedySearchNode gsn = heap.poll();

            // If node visits some tag on our visited list, continue. Otherwise,
            // if it visits a tag we have NOT yet visited, add that to the list.
            // NOTE: We need to be able to add finishing steps. I don't like our
            // old "detect when near the goal" method, so now would be a good
            // time to build something better. Like, I don't know, a notice
            // that we should be stopping "at" our goal?
        }

        return tree;
    }
    // ========================================================================


    /** Perform a hybrid search combining both DFS and best-first search
     *  strategies. Use DFS strategy to choose next node to expand, but BFS
     *  to evaluate when it's time to backtrack instead of mindlessly pursuing
     *  a bad avenue of search.
     **/
    int nodesExpanded;
    double penalty;
    private PriorityQueue<Node<Behavior> > heap =
        new PriorityQueue<Node<Behavior> >(10, new BNComparator());
    private PriorityQueue<GreedySearchNode> gsnHeap =
        new PriorityQueue<GreedySearchNode>(10, new GSNComparator());

    public Node<Behavior> hybridSearch(ArrayList<double[]> starts,
                                       double goal[])
    {
        // Initialize search
        ArrayList<Double> dists = new ArrayList<Double>();
        for (double[] s: starts)
            dists.add(0.0);

        gsnHeap.clear();
        Node<Behavior> behaviorNode = new Node<Behavior>(new Behavior(starts,
                                                                      dists,
                                                                      null,
                                                                      null));
        GreedySearchNode node = new GreedySearchNode(behaviorNode);
        gsnHeap.add(node);

        // Reset state from last search
        soln = null;
        solnScore = Double.MAX_VALUE;

        // Keep expanding the top node until we choose a node ending at the
        // goal. This is as good as we can expect to do, provided that our
        // scoring heuristic is admissible and consistent.
        while (gsnHeap.size() > 0) {
            node = gsnHeap.poll();

            // If this is close to our goal, return it!
            if (node.node.data.law instanceof DriveTowardsTag)
                return node.node;

            // Don't search TOO deep
            if (node.node.depth >= searchDepth)
                continue;

            // Trigger DriveTowardsTag finishing step if we are
            // sufficiently close to the goal.
            // XXX We need to visit this condition step. How do we set this
            // threshold? Why?
            double pct = node.node.data.getPctNearGoal(gm, wf, numSamples);
            double ARRIVAL_RATE_THRESH = 0.5; // XXX
            //if (pct >= ARRIVAL_RATE_THRESH) {
            if (node.node.data.tagID == goalTag.getID()) {
                gsnHeap.add(new GreedySearchNode(finishPlan(node.node, goalTag, pct)));
                continue;
            }

            // We're not done yet! Greedily pursue the next child in the
            // pecking order of possible choices. To do this, first do a
            // "perfect sensing" forward simulation to generate possible
            // actions to take. Then, use heuristic measures (assuming
            // the perfect sensing will hold true) to pick the next path
            // forwards. Actually simulate the act of following this route,
            // and add it to the heap. Additionally, add an (updated) entry
            // for the current search node to the heap, provided there are still
            // children to visit. If we have exhausted the list of children,
            // do not bother adding it back.
            if (!node.madeChildren()) {
                ArrayList<Behavior> records = generateChildren(node.node);
                node.addSortedChildren(records, new GreedyChildComparator());
            }

            // Add a child if it exists.
            if (node.hasNextChild()) {
                Behavior next = node.getNextChild();

                // Simulation.
                Behavior b = simulateBehavior(next, node.node);
                b.tagID = next.tagID;

                // Add to heap if non-null
                if (b != null) {
                    GreedySearchNode nextNode = new GreedySearchNode(node.node.addChild(b));
                    gsnHeap.add(nextNode);
                }
            }

            // If any children STILL exist, add the main node back, too
            if (node.hasNextChild()) {
                gsnHeap.add(node);
            }
        }

        return null;
    }

    /** Perform a heap-based search of the space. Always pursue the most
     *  promising candidate node.
     **/
    public Node<Behavior> heapSearch(ArrayList<double[]> starts,
                                     double goal[])
    {
        nodesExpanded = 0;
        penalty = 1.0;


        // Initialize search
        ArrayList<Double> dists = new ArrayList<Double>();
        for (double[] s: starts)
            dists.add(0.0);

        heap.clear();
        Behavior tb = new Behavior(starts, dists, null, null);
        Node<Behavior> node = new Node<Behavior>(tb);
        heap.add(node);

        // Reset state from last search
        soln = null;
        solnScore = Double.MAX_VALUE;

        MonteCarloBot mcb;

        // Keep expanding the top node until we pluck a node off the tree that
        // ends at the goal. This is as good as we can expect to do, provided
        // that our scoring heuristic is admissible and consistent
        while (heap.size() > 0) {
            node = heap.poll();
            System.out.printf("SCORE: %f\n", node.data.getBestScore(gm, wf, node.data.prob, 0));

            // If this is close to our goal, return it!
            if (node.data.law instanceof DriveTowardsTag)
                return node;

            // Don't search TOO deep...
            if (node.depth >= searchDepth)
                continue;


            // Minimum threshold for closeness to goal. Trigger "last instruction"
            double pct = node.data.getPctNearGoal(gm, wf, numSamples);
            double ARRIVAL_RATE_THRESH = 0.1;
            if (pct >= ARRIVAL_RATE_THRESH) {
                //System.out.printf("XXXXXXXXXXXXXXXXXXXXXXXXXX\n");

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
                        // Find where we are and how much we've driven to get there
                        xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                        distances.add(mcb.getTrajectoryLength());
                    }

                    //Node<Behavior> newNode = node.addChild(new Behavior(xyts, distances, dtt, new NearTag(params)));
                    Behavior b = new Behavior(xyts, distances, dtt, new NearTag(params));
                    if (node.data != null)
                        b.prob = Math.min(pct, node.data.prob);
                        //b.prob = node.data.prob;
                    Node<Behavior> newNode = node.addChild(b);
                    System.out.printf("\tSCORE: %f\n", b.getBestScore(gm, wf, b.prob, 0));
                    heap.add(newNode);
                }

                //soln = node;
                //solnScore = soln.data.getBestScore(gm, wf, numSamples, depth);
                //solnScore = soln.data.getBestScore(gm, wf, soln.data.prob, depth);
                continue;
            }

            watch.start("expansion");
            // XXX How would we incorporate more laws?
            // Expansion! Forward simulate to see what our possible control
            // might do.
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
            // Special case: If we are at the first step of our plan, we also consider
            // turning in place.
            if (node.depth == 0) {
                double[] yaws = new double[] {Math.PI/2, Math.PI, 3*Math.PI/2};
                for (double yaw: yaws) {
                    HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
                    params.put("direction", new TypedValue((byte)1));  // Left turn in place
                    params.put("yaw", new TypedValue(yaw));
                    params.put("no-lcm", new TypedValue(0));
                    HashMap<String, TypedValue> params2 = new HashMap<String, TypedValue>();
                    params2.put("yaw", new TypedValue(yaw));
                    params2.put("no-lcm", new TypedValue(0));

                    ArrayList<double[]> xyts = new ArrayList<double[]>();
                    dists = new ArrayList<Double>();
                    mcb = new MonteCarloBot(sw);
                    for (int i = 0; i < numExploreSamples; i++) {
                        Turn turn = new Turn(params);
                        RotationTest rt = new RotationTest(params2);
                        Behavior.XYTPair pair = node.data.randomXYT();
                        mcb.init(turn, rt, pair.xyt, pair.dist);
                        mcb.simulate();
                        // Only consider plans that actually "work"
                        if (mcb.success()) {
                            xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                            dists.add(mcb.getTrajectoryLength());
                        }
                    }

                    // XXX TURN
                    //recs.add(new Behavior(xyts, dists, new Turn(params), new RotationTest(params2)));
                }
            }
            watch.stop();

            watch.start("simulation");
            for (Behavior rec: recs) {
                mcb = new MonteCarloBot(sw);
                // Try multiple simulations to evaluate each possible step
                ArrayList<double[]> xyts = new ArrayList<double[]>();
                ArrayList<Double> distances = new ArrayList<Double>();
                for (int i = 0; i < numSamples; i++) {
                    Behavior.XYTPair pair = node.data.randomXYT();
                    mcb.init(rec.law, (ConditionTest)rec.test.copyCondition(), pair.xyt, pair.dist);
                    //mcb.simulate(70.0); // XXX How to choose?
                    mcb.simulate(300.0); // XXX
                    //System.out.printf(".");
                    if (vw != null) {
                        VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                        vb.setDrawOrder(-500);
                        vb.addBack(mcb.getVisObject());
                        vb.swap();
                    }
                    // Why do we only count success? Because
                    if (mcb.success()) {
                        // Find where we are and how much we've driven to get there
                        xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                        distances.add(mcb.getTrajectoryLength());
                    }
                }
                //System.out.printf("\n");
                if (xyts.size() < 1)
                    continue;
                Behavior behavior = new Behavior(xyts,
                                                 distances,
                                                 rec.law,
                                                 rec.test.copyCondition());
                if (node.data != null)
                    behavior.prob = node.data.prob;
                behavior.prob *= rec.prob;
                System.out.printf("\tSCORE: %f\n", behavior.getBestScore(gm, wf, behavior.prob, 0));
                //Node<Behavior> newNode = node.addChild(new Behavior(xyts, distances, rec.law, (ConditionTest)rec.test.copyCondition()));
                Node<Behavior> newNode = node.addChild(behavior);
                heap.add(newNode);
                nodesExpanded++;
            }
            watch.stop();
        }

        return null;
    }

    /** Do a depth first search for the best set of laws to follow */
    private SimAprilTag goalTag;
    private Node<Behavior> soln;
    private double solnScore;
    private Node<Behavior> dfsSearch(ArrayList<double[]> starts, double[] goal)
    {
        nodesExpanded = 0;
        penalty = 1.0;
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
        // Prune out solutions that are worse than our best so far.
        //double nodeScore = node.data.getBestScore(gm, wf, numSamples, depth);
        double nodeScore = node.data.getBestScore(gm, wf, node.data.prob, depth, penalty);
        if (nodeScore > solnScore) {
            System.out.printf("--PRUNED: [%f < %f] --\n", solnScore, nodeScore);
            return;
        }

        // This must have a better score than we currently have, because we got past
        // the pruning filter. Thus, this is a better solution.
        double pct = node.data.getPctNearGoal(gm, wf, numSamples);
        double ARRIVAL_RATE_THRESH = 0.1;
        if (pct >= ARRIVAL_RATE_THRESH) {
            //System.out.printf("XXXXXXXXXXXXXXXXXXXXXXXXXX\n");

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
                    // Find where we are and how much we've driven to get there
                    xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                    distances.add(mcb.getTrajectoryLength());
                }

                Behavior b = new Behavior(xyts, distances, dtt, new NearTag(params));
                if (node.data != null) {
                    b.prob = Math.min(pct, node.data.prob);
                    //b.prob = node.data.prob;
                }
                Node<Behavior> newNode = node.addChild(b);
                node = newNode;
            }

            soln = node;
            //solnScore = soln.data.getBestScore(gm, wf, numSamples, depth);
            System.out.println(soln.data.prob);
            solnScore = soln.data.getBestScore(gm, wf, soln.data.prob, depth);
            return;
        }

        // Don't search beyond our max depth.
        if (depth >= maxDepth) {
            //System.out.printf("--TOO DEEP--\n");
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
        watch.start("expansion");
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

        // Special case: If we are at the first step of our plan, we also consider
        // turning in place.
        if (depth == 0) {
            double[] yaws = new double[] {Math.PI/2, Math.PI, 3*Math.PI/2};
            for (double yaw: yaws) {
                HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
                params.put("direction", new TypedValue((byte)1));  // Left turn in place
                params.put("yaw", new TypedValue(yaw));
                params.put("no-lcm", new TypedValue(0));
                HashMap<String, TypedValue> params2 = new HashMap<String, TypedValue>();
                params2.put("yaw", new TypedValue(yaw));
                params2.put("no-lcm", new TypedValue(0));

                ArrayList<double[]> xyts = new ArrayList<double[]>();
                ArrayList<Double> dists = new ArrayList<Double>();
                mcb = new MonteCarloBot(sw);
                for (int i = 0; i < numExploreSamples; i++) {
                    Turn turn = new Turn(params);
                    RotationTest rt = new RotationTest(params2);
                    Behavior.XYTPair pair = node.data.randomXYT();
                    mcb.init(turn, rt, pair.xyt, pair.dist);
                    mcb.simulate();
                    // Only consider plans that actually "work"
                    if (mcb.success()) {
                        xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                        dists.add(mcb.getTrajectoryLength());
                    }
                }

                // XXX
                //recs.add(new Behavior(xyts, dists, new Turn(params), new RotationTest(params2)));
            }
        }
        watch.stop();

        // Test our record/law pairs based on tag distance to goal
        // and (heuristic warning!) estimated distance traveled.
        Collections.sort(recs, new BehaviorSearchComparator());
        mcb = new MonteCarloBot(sw);
        for (Behavior rec: recs) {
            if (debug) {
                System.out.printf("|");
                for (int i = 0; i < depth+1; i++) {
                    System.out.printf("==");
                }
                System.out.printf("%s", rec.toString());
            }

            // Prune out solutions that are worse than our best so far.
            // This is an early attempt at pruning to avoid spending more time
            // simulating many branches.
            // This is probably NOT a great thing to do, since we don't have
            // all that many samples.
            double recScore = rec.getBestScore(gm, wf, rec.prob, depth+1, penalty);
            if (solnScore < recScore) {
                System.out.printf("--EARLY PRUNED: [%f < %f]--\n", solnScore, recScore);
                continue;
            }
            watch.start("simulation");

            // Try multiple simulations to evaluate the step
            ArrayList<double[]> xyts = new ArrayList<double[]>();
            ArrayList<Double> distances = new ArrayList<Double>();
            for (int i = 0; i < numSamples; i++) {
                Behavior.XYTPair pair = node.data.randomXYT();
                mcb.init(rec.law, (ConditionTest)rec.test.copyCondition(), pair.xyt, pair.dist);
                //mcb.simulate(70.0); // XXX How to choose?
                mcb.simulate(300.0); // XXX
                //System.out.printf(".");
                if (vw != null) {
                    VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                    vb.setDrawOrder(-500);
                    vb.addBack(mcb.getVisObject());
                    vb.swap();
                }
                // Why do we only count success? Because
                if (mcb.success()) {
                    // Find where we are and how much we've driven to get there
                    xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                    distances.add(mcb.getTrajectoryLength());
                }
            }
            //System.out.printf("\n");
            if (xyts.size() < 1)
                continue;
            Behavior behavior = new Behavior(xyts,
                                             distances,
                                             rec.law,
                                             rec.test.copyCondition());
            if (node.data != null)
                behavior.prob = node.data.prob;
            behavior.prob *= rec.prob;
            //Node<Behavior> newNode = node.addChild(new Behavior(xyts, distances, rec.law, (ConditionTest)rec.test.copyCondition()));
            Node<Behavior> newNode = node.addChild(behavior);
            watch.stop();

            nodesExpanded++;
            dfsHelper(newNode, goal, depth+1, maxDepth);
        }

        return;
    }
}
