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
    int searchDepth = Util.getConfig().requireInt("monte_carlo.max_search_depth");
    int numExploreSamples = Util.getConfig().requireInt("monte_carlo.num_exploration_samples");
    int numSamples = Util.getConfig().requireInt("monte_carlo.num_evaluation_samples");

    WavefrontPlanner wfp;
    GridMap gm;
    float[] wf = null;

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

        public Behavior peekNextChild()
        {
            assert (hasNextChild());
            return futureChildren.get(nextChildIdx);
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

            double awfdist = a.node.data.getMeanDistToGoal(gm, wf);
            double bwfdist = b.node.data.getMeanDistToGoal(gm, wf);

            double diff = Math.abs(ascore - bscore);
            double maxDepth = Math.max(a.node.depth, b.node.depth);
            // If scores are sufficiently close, treat as equivalent and
            // order in favor of actual distance traveled instead of
            // estimated
            // XXX Change epsilon here for analysis! Don't want depth dependent.
            // XXX See old Ed emails.
            if (diff < EPSILON) {
                if (awfdist < bwfdist) {
                    return -1;
                } else if (awfdist > bwfdist) {
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

    private class SpanningTreeChildComparator implements Comparator<Behavior>
    {
        public int compare(Behavior a, Behavior b)
        {
            return 0;   // Order does not matter, since we make them all
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
            double da = awf;// + abonus;   // Multiplier on bonuses?
            double db = bwf;// + bbonus;

            // A bit of a hack to force early expansion of turns
            //if ((a.law instanceof Orient) && !(b.law instanceof Orient))
            //    return -1;
            //else if (!(a.law instanceof Orient) && (b.law instanceof Orient))
            //    return 1;

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
        params.put("side", new TypedValue(-1));
        params.put("distance", new TypedValue((double)0.75));
        controls.add(new FollowWall(params));
        params.put("side", new TypedValue(1));
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

        System.out.println("LAZILY EXPANDED: "+nodesLazilyExpanded);
        System.out.println("FULLY EXPANDED: "+nodesFullyExpanded);

        // Trace back behaviors to reach said node
        while (soln != null && soln.parent != null) {
            behaviors.add(soln.data);
            soln = soln.parent;
        }

        Collections.reverse(behaviors);
        if (behaviors.size() > 0) {
            Behavior last = behaviors.get(behaviors.size()-1);
            System.out.printf("FINAL SCORE: %f\n", last.getBestScore(gm, wf, last.prob, 0));
        }

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
    private Behavior finishPlan(Node<Behavior> node,
                                SimAprilTag goalTag)
    {
        HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
        //params.put("id", new TypedValue(goalTag.getID()));
        params.put("class", new TypedValue(tagdb.correctClass(goalTag.getID())));
        DriveTowardsTag dtt = new DriveTowardsTag(params);
        //params.put("distance", new TypedValue(0.1));    // XXX What should this be?

        ArrayList<double[]> startXYTs = new ArrayList<double[]>();
        ArrayList<double[]> endXYTs = new ArrayList<double[]>();
        ArrayList<double[]> startOdoms = new ArrayList<double[]>();
        ArrayList<double[]> endOdoms = new ArrayList<double[]>();
        ArrayList<Double> startDists = new ArrayList<Double>();
        ArrayList<Double> endDists = new ArrayList<Double>();


        MonteCarloBot mcb = new MonteCarloBot(sw);
        for (int i = 0; i < numSamples; i++) {
            Behavior.XYTPair pair = node.data.randomXYT(); // Theoretical?
            mcb.init(dtt,
                     new Stabilized(params),
                     pair.endXYT,
                     pair.endOdom,
                     pair.dist);
            watch.start("sim");
            mcb.simulate(10.0); // XXX Another magic number
            watch.stop();
            if (vw != null) {
                VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                vb.setDrawOrder(-500);
                vb.addBack(mcb.getVisObject());
                vb.swap();
            }
            // Find where we are and how much we've driven to get there
            startXYTs.add(pair.endXYT);
            endXYTs.add(LinAlg.matrixToXYT(mcb.getPose()));
            startOdoms.add(pair.endOdom);
            endOdoms.add(LinAlg.matrixToXYT(mcb.getOdom()));
            startDists.add(pair.dist);
            endDists.add(mcb.getTrajectoryLength());
        }

        // The behavior begins where the last one ended.
        Behavior b = new Behavior(startXYTs,
                                  endXYTs,
                                  startOdoms,
                                  endOdoms,
                                  startDists,
                                  endDists,
                                  dtt,
                                  new Stabilized(params));

        if (node.data != null) {
            b.prob = node.data.prob;
            b.myprob = 1.0;
        }
        b.tagID = goalTag.getID(); // XXX
        return b;
        //Node<Behavior> newNode = node.addChild(b);
        //System.out.printf("\tSCORE: %f\n", b.getBestScore(gm, wf, b.prob, 0));
        //return newNode;
    }

    private ArrayList<Behavior> generateChildren(Node<Behavior> node)
    {
        double[] yaws = new double[] {0, Math.PI/2, Math.PI, -Math.PI/2};
        //double[] yaws = new double[] {};
        return generateChildren(node, yaws);
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
     *
     *  @param node     Node the start from
     *  @param yaws     Additional turn-in-place commands to potentially sim
     *
     *  @return A list of behavior/condition tuples
     **/
    private ArrayList<Behavior> generateChildren(Node<Behavior> node, double[] yaws) {
        ArrayList<Behavior> recs = new ArrayList<Behavior>();

        ArrayList<Node<Behavior> > startNodes = new ArrayList<Node<Behavior> >();
        startNodes.add(node);

        boolean isTurn = false;
        if (node.data.law != null)
            isTurn = node.data.law instanceof Orient;

        // Special case: we also consider turning in place at the beggining
        // phase of planning. Don't chain turns
        if ((node.depth == 0 || true) && !isTurn) {
            for (double yaw: yaws) {
                HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
                params.put("yaw", new TypedValue(yaw));
                params.put("no-lcm", new TypedValue(0));

                ArrayList<double[]> startXYTs = new ArrayList<double[]>();
                ArrayList<double[]> xyts = new ArrayList<double[]>();
                ArrayList<double[]> startOdoms = new ArrayList<double[]>();
                ArrayList<double[]> endOdoms = new ArrayList<double[]>();
                ArrayList<Double> dists = new ArrayList<Double>();
                ArrayList<Double> startDists = new ArrayList<Double>();
                MonteCarloBot mcb = new MonteCarloBot(sw);
                for (int i = 0; i < numExploreSamples; i++) {
                    Orient orient = new Orient(params);
                    Stabilized stable = new Stabilized(params);
                    Behavior.XYTPair pair = node.data.randomXYT();
                    mcb.init(orient,
                             stable,
                             pair.endXYT,
                             pair.endOdom,
                             pair.dist);
                    watch.start("sim");
                    mcb.simulate(10.0);
                    watch.stop();
                    // Only consider plans that actually "work"
                    //if (mcb.success()) {
                    startXYTs.add(pair.endXYT);
                    xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
                    startOdoms.add(pair.endOdom);
                    endOdoms.add(LinAlg.matrixToXYT(mcb.getOdom()));
                    dists.add(mcb.getTrajectoryLength());
                    startDists.add(pair.dist);
                    //}
                }

                if (xyts.size() < 1) {
                    System.out.printf("Failed to turn to %f\n", yaw);
                    continue;
                }

                Behavior b = new Behavior(startXYTs,
                                          xyts,
                                          startOdoms,
                                          endOdoms,
                                          startDists,
                                          dists,
                                          new Orient(params),
                                          new Stabilized(params));
                b.myprob = 0.999;   // Some penalty for turning
                b.prob = node.data.prob * b.myprob;    // XXX Not perfect! Fudge
                recs.add(b);
            }
        }

        for (Node<Behavior> startNode: startNodes) {
            for (FollowWall law: controls) {
                MonteCarloBot mcb = new MonteCarloBot(sw);
                for (int i = 0; i < numExploreSamples; i++) {
                    Behavior.XYTPair pair = startNode.data.randomXYT();
                    mcb.init(law,
                             null,
                             startNode.data.theoreticalXYT.endXYT,
                             startNode.data.theoreticalXYT.endXYT,  // Doesn't matter here
                             startNode.data.theoreticalXYT.dist);
                    watch.start("sim");
                    mcb.simulate(true); // This will always timeout. PERFECT ADDED LATE
                    watch.stop();
                }
                for (Behavior rec: mcb.tagRecords.values()) {
                    recs.add(rec);
                }
            }

            // Handle finishing steps. Note: We use visual servoing towards our
            // ACTUAL goal, necessitating tag matches. In reality, we would like
            // to servo towards something of type "label"
            if (startNode.data.tagID > 0 && !(startNode.data.law instanceof DriveTowardsTag)) {
                SimAprilTag tag = getTag(node.data.tagID);
                Behavior next = finishPlan(node, tag);
                recs.add(next);
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
        ArrayList<double[]> startXYTs = new ArrayList<double[]>();
        ArrayList<double[]> xyts = new ArrayList<double[]>();
        ArrayList<double[]> startOdoms = new ArrayList<double[]>();
        ArrayList<double[]> endOdoms = new ArrayList<double[]>();
        ArrayList<Double> distances = new ArrayList<Double>();
        ArrayList<Double> startDists = new ArrayList<Double>();
        int successCount = 0;
        for (int i = 0; i < numSamples; i++) {
            Behavior.XYTPair pair = node.data.randomXYT();
            ConditionTest test = b.test.copyCondition();
            mcb.init(b.law,
                     test,
                     pair.endXYT,
                     pair.endOdom,
                     pair.dist);
            //mcb.simulate(70.0); // XXX How to choose?
            double time = 300.0;
            if (b.law instanceof Orient || b.law instanceof DriveTowardsTag)
                time = 10.0;
            watch.start("sim");
            mcb.simulate(time); // XXX
            watch.stop();
            if (vw != null) {
                VisWorld.Buffer vb = vw.getBuffer("debug-DFS");
                vb.setDrawOrder(-500);
                vb.addBack(mcb.getVisObject());
                vb.swap();
            }
            if (test instanceof ClassificationCounterTest) {
                ClassificationCounterTest cct = (ClassificationCounterTest)test;
                if (cct.conditionMetCorrectly())
                    successCount++;
            } else {
                if (mcb.success())
                    successCount++; // Meh
            }

            // We only count success because ...? XXX
            //if (mcb.success()) {
            // Find where we are and how much we've driven to get there
            startXYTs.add(pair.endXYT);
            xyts.add(LinAlg.matrixToXYT(mcb.getPose()));
            startOdoms.add(pair.endOdom);
            endOdoms.add(LinAlg.matrixToXYT(mcb.getOdom()));
            startDists.add(pair.dist);
            distances.add(mcb.getTrajectoryLength());
            //}
        }
        if (xyts.size() < 1)
            return null;    // Complete and utter failure of execution
        Behavior behavior = new Behavior(startXYTs,
                                         xyts,
                                         startOdoms,
                                         endOdoms,
                                         startDists,
                                         distances,
                                         b.law,
                                         b.test.copyCondition());
        behavior.successCount = successCount;
        double successRate = behavior.getPctNearTheoretical();
        if (b.law instanceof Orient || b.law instanceof DriveTowardsTag)
            behavior.prob = node.data.prob * successRate;
        else
            behavior.prob = successRate;

        // XXX So do we even USE our sample points due to this?
        //if (node.data != null)
        //    behavior.prob = node.data.prob;
        //behavior.prob *= behavior.myprob;
        behavior.tagID = b.tagID; //node.data.tagID;   // COPY OVER TAG ID! Awful bookkeeping
        behavior.theoreticalXYT = b.theoreticalXYT.copy();
        //System.out.printf("\t%s SCORE: %f\n", behavior.law.toString(), behavior.getBestScore(gm, wf, behavior.prob, 0));

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

    public Tree<Behavior> buildSpanningTree(int tagID)
    {
        // By default, just run forever
        return buildSpanningTree(tagID, true, Long.MAX_VALUE);
    }

    /** Build a spanning tree describing trajectories from the given to all
     *  others.
     *
     *  @param tagID        ID of tag to build from
     *  @param timeout_us   Amount of time to spend building tree.
     *
     *  @return A spanning tree of the best routes to each goal
     **/
    public Tree<Behavior> buildSpanningTree(int tagID, boolean addNeighbors, long timeout_us)
    {
        boolean CONNECT_NEIGHBORS = true;
        watch = new Stopwatch();
        watch.start("spanning-tree");
        long start_utime = TimeUtil.utime();
        SimAprilTag tag = getTag(tagID);
        assert (tag != null);

        double[] xyt_0 = LinAlg.matrixToXYT(tag.getPose());
        xyt_0[2] = 0;

        // Initialize the tree as if we were facing down the X-axis at the
        // tag location. It is up to our search to escape efficiently.
        // Is there a place for turn-in-place? I expect yes, and that we'll
        // want that.
        Tree<Behavior> tree = new Tree<Behavior>(new Behavior(xyt_0,
                                                              xyt_0,
                                                              xyt_0,
                                                              xyt_0,
                                                              0, 0,
                                                              null, null));

        // Find the best routes to all other locations simultaneously. Reward
        // a combination of distance traveled and arrival rate. Basically,
        // normal scoring without wavefront predictions
        PriorityQueue<GreedySearchNode> heap =
            new PriorityQueue<GreedySearchNode>(10, new SpanningTreeComparator());
        heap.add(new GreedySearchNode(tree.root));

        Set<Integer> visitedTags = new HashSet<Integer>();
        visitedTags.add(tag.getID());

        while (heap.size() > 0 && (TimeUtil.utime() - start_utime) < timeout_us) {
            GreedySearchNode gsn = heap.poll();

            // If node visits some tag on our visited list, continue. Otherwise,
            // if it visits a tag we have NOT yet visited, add it to the list
            // and add the node to our tree. (Given the tight integration
            // of Tree nodes with GSNs, tree management could be weird)
            int currTagID = gsn.node.data.tagID;

            // XXX Can we maintain such a visited list, after all? Maybe not,
            // since visiting a tag from a new angle might be useful...try
            // it for now and see.
            //
            // This gives us huge time savings and seems necessary to allow the
            // search to proceed usefully in some capacity. It also prevents us
            // from linking together visually servoed commands...
            if (visitedTags.contains(currTagID) && !(gsn.node.data.law instanceof DriveTowardsTag))
                continue;
            if (currTagID >= 0 && !visitedTags.contains(currTagID)) {
                // We are at a new tag! Add a finishing step to the tree.
                // Don't bother adding this to the heap
                visitedTags.add(currTagID);
                Behavior lastStep = finishPlan(gsn.node,
                                               getTag(currTagID));
                Node<Behavior> node = gsn.node.addChild(lastStep);
                heap.add(new GreedySearchNode(node));
                //continue;
            }

            // Next, generate the children for this node and toss them onto
            // the heap. Only generate children that don't end at the closed
            // list.
            watch.start("generate-children");
            ArrayList<Behavior> behaviors = generateChildren(gsn.node);
            gsn.addSortedChildren(behaviors, new SpanningTreeChildComparator());
            watch.stop();

            watch.start("simulate-children");
            while (gsn.hasNextChild()) {
                Behavior next = gsn.getNextChild();
                // XXX Same problem as earlier. Make the concession for the sake
                // of rapid exploration?
                if (visitedTags.contains(next.tagID))
                    continue;

                // Simulation
                //Behavior b = simulateBehavior(next, gsn.node);
                //b.tagID = next.tagID;
                Behavior b = next;
                b.prob *= gsn.node.data.prob;

                if (b != null) {
                    Node<Behavior> node = gsn.node.addChild(b);
                    GreedySearchNode nextGSN = new GreedySearchNode(node);

                    // Handle turns early
                    if (b.law instanceof Orient) {
                        watch.start("turns");
                        ArrayList<Behavior> turnBehaviors = generateChildren(nextGSN.node);
                        nextGSN.addSortedChildren(turnBehaviors, new SpanningTreeChildComparator());
                        while (nextGSN.hasNextChild()) {
                            Behavior turnNext = nextGSN.getNextChild();
                            // XXX One more visitation skip step
                            if (visitedTags.contains(turnNext.tagID))
                                continue;

                            Behavior tb = turnNext;
                            tb.prob *= nextGSN.node.data.prob;
                            Node<Behavior> nextNode = nextGSN.node.addChild(tb);

                            // Add immediately adjacent neighbors
                            if (addNeighbors && gsn.node.depth == 0 && tb.tagID >= 0 && !visitedTags.contains(tb.tagID)) {
                                // XXX Does this count as visiting, yet?
                                visitedTags.add(tb.tagID);
                                Behavior lastStep = finishPlan(nextNode,
                                                               getTag(tb.tagID));
                                heap.add(new GreedySearchNode(nextNode.addChild(lastStep)));
                            }

                            heap.add(new GreedySearchNode(nextNode));
                        }
                        watch.stop();
                        continue;
                    }

                    // Add immediately adjacent neighbors
                    if (addNeighbors && gsn.node.depth == 0 && node.data.tagID >= 0 && !visitedTags.contains(node.data.tagID)) {
                        // XXX Does this count as visiting, yet?
                        visitedTags.add(node.data.tagID);
                        Behavior lastStep = finishPlan(node,
                                                       getTag(node.data.tagID));
                        heap.add(new GreedySearchNode(node.addChild(lastStep)));
                    }

                    heap.add(nextGSN);
                }
            }
            watch.stop();
        }

        long timeDiff = TimeUtil.utime() - start_utime;
        System.out.printf("Spent %d of %d searching\n", timeDiff, timeout_us);
        System.out.printf("Heap size %d\n", heap.size());

        // Clean up non-terminal leaves
        ArrayList<Node<Behavior> > leaves = tree.getLeaves();
        for (Node<Behavior> leaf: leaves) {
            if (leaf.data.law instanceof DriveTowardsTag)
                continue;
            Node<Behavior> curr = leaf;
            Node<Behavior> next = curr.parent;
            while (curr.children.size() == 0 && next != null) {
                next.removeChild(curr);
                curr = next;
                next = curr.parent;
                if (curr.data.law instanceof DriveTowardsTag)
                    break;
            }
        }

        watch.stop();
        watch.print();
        return tree;
    }
    // ========================================================================
    // XXX Old DFS things. Still used some places
    private SimAprilTag goalTag;
    private Node<Behavior> soln;
    private double solnScore;

    /** Perform a hybrid search combining both DFS and best-first search
     *  strategies. Use DFS strategy to choose next node to expand, but BFS
     *  to evaluate when it's time to backtrack instead of mindlessly pursuing
     *  a bad avenue of search.
     **/
    public int nodesLazilyExpanded;
    public int nodesFullyExpanded;
    double penalty;
    private PriorityQueue<GreedySearchNode> gsnHeap =
        new PriorityQueue<GreedySearchNode>(10, new GSNComparator());

    public Node<Behavior> hybridSearch(ArrayList<double[]> starts,
                                       double goal[])
    {
        nodesLazilyExpanded = 0;
        nodesFullyExpanded = 0;
        // Initialize search
        ArrayList<Double> dists = new ArrayList<Double>();
        for (double[] s: starts)
            dists.add(0.0);

        gsnHeap.clear();
        Node<Behavior> behaviorNode = new Node<Behavior>(new Behavior(starts,
                                                                      starts,
                                                                      starts,
                                                                      starts,
                                                                      dists,
                                                                      dists,
                                                                      null,
                                                                      null));
        GreedySearchNode node = new GreedySearchNode(behaviorNode);
        gsnHeap.add(node);

        HashSet<Integer> closedList = new HashSet<Integer>();
        HashSet<Integer> closedChildren = new HashSet<Integer>();

        // Reset state from last search
        soln = null;
        solnScore = Double.MAX_VALUE;

        // Keep expanding the top node until we choose a node ending at the
        // goal. This is as good as we can expect to do, provided that our
        // scoring heuristic is admissible and consistent.
        while (gsnHeap.size() > 0) {
            node = gsnHeap.poll();

            // If this is close to our goal, return it!
            if (node.node.data.law instanceof DriveTowardsTag) {
                if (goalTag.getID() == node.node.data.tagID) {
                    return node.node;
                }
            }

            // Don't search TOO deep
            if (node.node.depth >= searchDepth)
                continue;

            // If we have already expanded some version of this node,
            // skip it. This isn't quite fair, as encountering a landmark
            // from some other direction might be advantageous, but it should
            // slim down the repetitions. Revisit later to investigate impact?
            // NOTE: The policy followed here MAY matter in the case of wall
            // followed. We might want to come back and change how that is handled.
            // For now, just greedily take the first such option. For our cases,
            // they should be morally equivalent.
            if (closedList.contains(node.node.data.tagID) &&
                !(node.node.data.law instanceof DriveTowardsTag))
            {
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
                nodesLazilyExpanded += records.size();
                node.addSortedChildren(records, new GreedyChildComparator());
            }

            // Add a child if it exists.
            if (node.hasNextChild()) {
                Behavior next = node.getNextChild();

                if (next != null && (!closedChildren.contains(next.tagID) || next.law instanceof DriveTowardsTag)) {
                    // Simulation.
                    Behavior b = simulateBehavior(next, node.node);
                    nodesFullyExpanded++;

                    b.tagID = next.tagID;

                    // Add to heap if non-null
                    if (b != null) {
                        GreedySearchNode nextNode = new GreedySearchNode(node.node.addChild(b));

                        gsnHeap.add(nextNode);
                        if (next.tagID > 0)
                            closedChildren.add(next.tagID);
                    }
                }
            }

            // If any children STILL exist, add the main node back, too
            if (node.hasNextChild()) {
                gsnHeap.add(node);
            } else if (node.node.data.tagID > 0) {
                closedList.add(node.node.data.tagID);
                closedChildren.add(node.node.data.tagID);
            }

        }

        return null;
    }

}
