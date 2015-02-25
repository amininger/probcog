package probcog.navigation;

import java.awt.Color;
import java.util.*;

import april.jmat.*;
import april.vis.*;
import april.util.*;

import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.util.*;

/** A graph structure with some wrinkles particular to this application.
 *  Could certainly be done generally given more time, but this is
 *  expedient.
 *
 *  Stores a directed graph (to be built from pieces taken from a tree
 *  of behaviors). "Nodes" exist at landmarks in the world. Edges connect
 *  nodes, but in a strange way...given the importance of starting state,
 *  incoming and outgoing edges must be connected in the node. This
 *  ensures that only valid control sequences are selected going forwards.
 */
public class BehaviorGraph
{
    static final double LAMBDA = Util.getConfig().requireDouble("monte_carlo.lambda");
    static final double WHEELBASE_M = 0.5;
    static final double XYT_DIST_M = Util.getConfig().requireDouble("monte_carlo.xyt_dist_m");
    static final double XYT_THETA_RAD = Math.toRadians(Util.getConfig().requireDouble("monte_carlo.xyt_theta_deg"));

    class DijkstraComparator implements Comparator<DijkstraNode>
    {
        public int compare(DijkstraNode a, DijkstraNode b)
        {
            if (a.score < b.score)
                return -1;
            else if (b.score < a.score)
                return 1;
            return 0;
        }
    }

    class DijkstraNode
    {
        public int nodeIdx = -1;            // Our current node
        public int edgeID = -1;             // The edge that brought us here
        public double prob = 1.0;
        public double dist = 0;
        public double distLeft = 0;
        public double score = 0;
        public DijkstraNode parent = null;

        private DijkstraNode() {};

        public DijkstraNode(int nodeIdx)
        {
            this.nodeIdx = nodeIdx;
        }

        /** Add a child to our search graph. The child documents the edgeID
         *  of the edge that took it to its presesnt location. The edge itself
         *  contains a behavior noting which node it is currently at.
         **/
        public DijkstraNode addChild(int edgeID, GridMap gm, float[] costMap)
        {
            assert (edges.containsKey(edgeID));
            Edge e = edges.get(edgeID);

            DijkstraNode dn = new DijkstraNode();
            dn.edgeID = edgeID;
            dn.nodeIdx = findNodeIdx(e.b.theoreticalXYT.endXYT);
            dn.prob = prob*e.b.myprob;
            dn.dist = dist+e.b.theoreticalXYT.myDist;

            if (gm != null) {
                double[] xy = nodes.get(dn.nodeIdx).xy;

                int ix = (int)(Math.floor((xy[0]-gm.x0)/gm.metersPerPixel));
                int iy = (int)(Math.floor((xy[1]-gm.y0)/gm.metersPerPixel));

                dn.distLeft = (double) (costMap[iy*gm.width + ix]);
            } else {
                dn.distLeft = 0;
            }
            dn.score = (dn.dist + dn.distLeft) - dn.prob*LAMBDA;
            //dn.score = (dn.dist + dn.distLeft);

            dn.parent = this;

            return dn;
        }

        public Edge getEdge()
        {
            if (edgeID < 0)
                return null;
            return edges.get(edgeID);
        }

        public Node getNode()
        {
            if (nodeIdx < 0)
                return null;
            return nodes.get(nodeIdx);
        }

        public int hashCode()
        {
            return (new Integer(edgeID)).hashCode();
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof DijkstraNode))
                return false;
            DijkstraNode dn = (DijkstraNode)o;
            return dn.edgeID == edgeID;
        }
    }

    HashMap<Integer, Integer> tagIDs = new HashMap<Integer, Integer>(); // id -> nodeidx
    ArrayList<Node> nodes = new ArrayList<Node>();   // Inefficient, but easy
    static class Node
    {
        public int tagID = -1;  // If this node counts as being at a tag,
                                // mark it as such
        double[] xy;

        // Inbound and outbound edges
        Set<Integer> edgesIn = new HashSet<Integer>();
        Set<Integer> edgesOut = new HashSet<Integer>();

        public Node(double[] xy)
        {
            this(-1, xy);   // -1 == no tag associated
        }

        public Node(int tagID, double[] xy)
        {
            this.tagID = tagID;
            this.xy = xy;
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof Node))
                return false;
            Node n = (Node)o;
            return xyEquals(xy, n.xy);
        }
    }

    static int edgeID = 0;
    Map<Integer, Edge> edges = new HashMap<Integer, Edge>();    // id -> edge
    static class Edge
    {
        public int id;
        public Behavior b;

        public Edge(Behavior b)
        {
            id = edgeID++;
            this.b = b.copyBehavior();
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof Edge))
                return false;
            Edge e = (Edge)o;
            return b.behaviorEquals(e.b);
        }
    }

    static private boolean xyEquals(double[] xy0, double[] xy1)
    {
        double dx = Math.abs(xy0[0] - xy1[0]);
        double dy = Math.abs(xy0[1] - xy1[1]);

        return Math.sqrt(dx*dx + dy*dy) < XYT_DIST_M;
    }

    static private boolean xytEquals(double[] xyt0, double[] xyt1)
    {
        double dx = Math.abs(xyt0[0] - xyt1[0]);
        double dy = Math.abs(xyt0[1] - xyt1[1]);
        double dt = Math.abs(MathUtil.mod2pi(xyt0[2] - xyt1[2]));
        dt = 0; // Ignore dt
        //System.out.printf("[%f %f %f]\n", dx, dy, dt);

        return Math.sqrt(dx*dx + dy*dy) < XYT_DIST_M && dt < XYT_THETA_RAD;  // XXX
    }

    /** Find the index of a matching node, if any exists. If none is found,
     *  returns -1, an invalid index.
     **/
    private int findNodeIdx(double[] xy)
    {
        for (int i = 0; i < nodes.size(); i++) {
            Node node = nodes.get(i);
            if (xyEquals(xy, node.xy))
                return i;
        }
        return -1;
    }

    private int findNodeIdxByID(int tagID)
    {
        if (tagIDs.containsKey(tagID))
            return tagIDs.get(tagID);
        return -1;
    }

    private Set<Integer> setIntersect(Set<Integer> s0, Set<Integer> s1)
    {
        Set<Integer> intersection = new HashSet<Integer>();
        for (Integer i: s0) {
            for (Integer j: s1) {
                if (i.equals(j))
                    intersection.add(i);
            }
        }
        return intersection;
    }

    public void addEdge(Behavior b)
    {
        assert (b != null);
        assert (b.law != null);
        assert (b.test != null);

        double[] startXYT = b.theoreticalXYT.startXYT;
        double[] endXYT = b.theoreticalXYT.endXYT;

        int tagID = -1;
        if (b.law instanceof DriveTowardsTag)
            tagID = b.tagID;

        int startIdx = findNodeIdx(startXYT);
        int endIdx = findNodeIdx(endXYT);

        // If nodes do not exist, create them
        if (startIdx < 0) {
            startIdx = nodes.size();
            nodes.add(new Node(startXYT));
        }
        if (endIdx < 0) {
            endIdx = nodes.size();
            nodes.add(new Node(tagID, endXYT));
        }

        // XXX
        if (tagID >= 0 && !tagIDs.containsKey(tagID)) {
            tagIDs.put(tagID, endIdx);
        }

        Node out = nodes.get(startIdx);
        Node in = nodes.get(endIdx);

        // Node might have been created already. Shouldn't ever have another
        // tag label, so this should be fine.
        if (tagID >= 0) {
            assert (in.tagID == tagID || in.tagID == -1);
            in.tagID = tagID;
        }

        Set<Integer> edgeset = setIntersect(out.edgesOut, in.edgesIn);
        Edge match = new Edge(b);    // ID wackiness when tons of matches
        for (Integer key: edgeset) {
            Edge edge = edges.get(key);
            if (edge.equals(match)) {
                match = edge;
                break;
            }
        }

        boolean addedout = out.edgesOut.add(match.id);
        boolean addedin = in.edgesIn.add(match.id);
        if (addedout && addedin)
            edges.put(match.id, match);
        else if (addedout || addedin)
            assert (false);
    }

    // XXX Eventually, want to change "startTag" to a specific
    // goal or orientation to match against graph structure.
    public ArrayList<Behavior> navigate(int startTag, int endTag, GridMap gm, VisWorld vw)
    {
        // Perform a Dijkstra search through the graph. MOAR NODES
        int startIdx = findNodeIdxByID(startTag);
        assert (startIdx >= 0);
        int endIdx = findNodeIdxByID(endTag);
        assert (endIdx >= 0);

        double[] goal = nodes.get(endIdx).xy;

        // Compute wavefront
        float[] costMap = null;
        if (gm != null) {
            WavefrontPlanner wfp = new WavefrontPlanner(gm, 0.0);
            costMap = wfp.getWavefront(null, goal);
        }

        DijkstraNode dn = new DijkstraNode(startIdx);

        HashSet<DijkstraNode> closedList = new HashSet<DijkstraNode>();
        PriorityQueue<DijkstraNode> queue =
            new PriorityQueue<DijkstraNode>(10, new DijkstraComparator());
        queue.add(dn);

        VisWorld.Buffer vb = null;
        if (vw != null) {
            vb = vw.getBuffer("debug-graph-nav");
        }

        while (queue.size() > 0) {
            dn = queue.poll();
            //System.out.printf("%f+%f - %f*%f = %f\n", dn.dist, dn.distLeft, LAMBDA, dn.prob, dn.score);

            // If this node reaches our goal, return a plan
            Node currNode = dn.getNode();
            assert (currNode != null);
            if (currNode.tagID == endTag) {
                if (vb != null) {
                    vb.swap();
                }
                return planHelper(dn);
            }

            // Pass over previously visited search nodes
            if (closedList.contains(dn)) {
                continue;
            }
            closedList.add(dn);

            // Generate children
            for (Integer edgeID: currNode.edgesOut) {
                queue.add(dn.addChild(edgeID, gm, costMap));

                if (vb != null) {
                    Edge e = edges.get(edgeID);
                    vb.addBack(new VisChain(LinAlg.xytToMatrix(e.b.theoreticalXYT.startXYT),
                                            new VzBox(new VzMesh.Style(Color.red))));
                }
            }

            if (vb != null) {
                vb.swap();
                TimeUtil.sleep(250);
            }
        }

        if (vb != null) {
            vb.swap();
        }

        return null; // Failure
    }

    private ArrayList<Behavior> planHelper(DijkstraNode dn)
    {
        ArrayList<Behavior> plan = new ArrayList<Behavior>();

        Node node = dn.getNode();
        Edge edge = dn.getEdge();

        // Walk back through our plan, creating a sequence of the appropriate
        // behaviors for planner consumption. In the event that the end point
        // orientations for a behavior to be included do NOT align with the
        // start points of another, first perform a corrective turn in place.
        DijkstraNode prev = dn;
        while (dn.parent != null) {
            edge = dn.getEdge();

            // Determine turn necessity. Remember, we are traversing the plan
            // BACKWARDS, so take this into account when generating a turn
            double[] prevXYT = prev.getEdge().b.theoreticalXYT.startXYT;
            double[] currXYT = edge.b.theoreticalXYT.endXYT;
            double myDist = edge.b.theoreticalXYT.myDist;
            double dist = edge.b.theoreticalXYT.dist;
            double dt = Math.abs(prevXYT[2] - currXYT[2]);
            if (dt > XYT_THETA_RAD) {
                HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
                params.put("yaw", new TypedValue(prevXYT[2]));
                params.put("no-lcm", new TypedValue(1));
                Orient orient = new Orient(params);
                Stabilized stable = new Stabilized(params);
                Behavior b = new Behavior(currXYT,
                                          prevXYT,
                                          currXYT,      // XXX Meaningless here, so just use perfect
                                          prevXYT,      // XXX Meaningless here, as well
                                          dist,
                                          dist+dt*(WHEELBASE_M/2),
                                          orient,
                                          stable);
                b.prob = edge.b.prob; // XXX
                plan.add(b); // XXX WIP
            }

            plan.add(edge.b.copyBehavior());
            prev = dn;
            dn = dn.parent;
        }
        plan.add(new Behavior(prev.getEdge().b.theoreticalXYT.startXYT,
                              prev.getEdge().b.theoreticalXYT.startXYT,
                              prev.getEdge().b.theoreticalXYT.startXYT,
                              prev.getEdge().b.theoreticalXYT.startXYT,
                              0,
                              0,
                              null,
                              null));

        Collections.reverse(plan);
        //System.out.println("Constructed plan of size: "+plan.size());
        //for (Behavior b: plan)
        //    System.out.println(b);
        //System.out.println("=====================");
        return plan;
    }

    /** Determine whether or not the graph is fully reachable. */
    public boolean isFullyReachable(VisWorld vw)
    {
        // Brute force: try to reach every node from every other one
        for (Integer startID: tagIDs.keySet()) {
            for (Integer endID: tagIDs.keySet()) {
                if (startID.equals(endID))
                    continue;
                System.out.printf("Testing reachability from %d to %d\n", startID, endID);
                if (navigate(startID, endID, null, vw) == null)
                    return false;
            }
        }
        return true;
    }

    // XXX DEBUG
    public void print()
    {
        System.out.println("TAG IDS: ");
        for (Integer id: tagIDs.keySet())
            System.out.printf("\t%d\n", id);
    }

}
