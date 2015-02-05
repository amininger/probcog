package probcog.navigation;

import java.awt.Color;
import java.util.*;

import april.jmat.*;
import april.vis.*;
import april.util.*;

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
    class DijkstraComparator implements Comparator<DijkstraNode>
    {
        public int compare(DijkstraNode a, DijkstraNode b)
        {
            if (a.score > b.score)
                return -1;
            else if (b.score > a.score)
                return 1;
            return 0;
        }
    }

    class DijkstraNode
    {
        public int nodeID = -1;
        public int edgeID = -1;
        public double score = 1.0;
        public DijkstraNode parent = null;

        private DijkstraNode() {};

        public DijkstraNode(int nodeID)
        {
            // XXX What about non-node based search/lookup? Arbitrary locations!
            this.nodeID = nodeID;
        }

        /** Add a child to our search graph. The child documents the edgeID
         *  of the edge that took it to its presesnt location. The edge itself
         *  contains a behavior noting which node it is currently at.
         **/
        public DijkstraNode addChild(int edgeID)
        {
            assert (edges.containsKey(edgeID));
            Edge e = edges.get(edgeID);

            DijkstraNode dn = new DijkstraNode();
            dn.edgeID = edgeID;
            dn.nodeID = e.b.tagID;   // If -1? We're currently saying this never happens
            dn.score = score*e.b.myprob;

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
            if (nodeID < 0)
                return null;
            return nodes.get(nodeID);
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

    //static int nodeID = 0;
    Map<Integer, Node> nodes = new HashMap<Integer, Node>();    // id -> node
    static class Node
    {
        public int id;

        // A mapping of inbound edges to outbound edges they can feed into.
        Map<Integer, Set<Integer> > in2out = new HashMap<Integer, Set<Integer> >();

        // Inbound and outbound edges
        Set<Integer> edgesIn = new HashSet<Integer>();
        Set<Integer> edgesOut = new HashSet<Integer>();

        public Node(int tagID)
        {
            id = tagID; // Will this be sufficient?
        }
    }

    static int edgeID = 0;
    Map<Integer, Edge> edges = new HashMap<Integer, Edge>();    // id -> edge
    static class Edge
    {
        public int id;
        public Behavior b;

        // XXX NEED TO KNOW START AND END POINTS ARE FOR LOOKUPS! Ends are
        // encoded in the behavior...starts need to be maintained separately
        public ArrayList<double[]> startXYTs = new ArrayList<double[]>();

        public Edge(Behavior b, double[] xyt)
        {
            id = edgeID++;
            this.b = b.copyBehavior();
            startXYTs.add(xyt);
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

    private boolean xytEquals(double[] xyt0, double[] xyt1)
    {
        double dx = Math.abs(xyt0[0] - xyt1[0]);
        double dy = Math.abs(xyt0[1] - xyt1[1]);
        double dt = Math.abs(MathUtil.mod2pi(xyt0[2] - xyt1[2]));
        //System.out.printf("[%f %f %f]\n", dx, dy, dt);

        return dx < 0.2 && dy < 0.2 && dt < Math.toRadians(5);  // XXX
    }

    /** Add a node to the graph */
    public void addNode(int tagID)
    {
        nodes.put(tagID, new Node(tagID)); // XXX Do we care about XY location?
    }

    /** Add an edge to the graph, or modify an existing edge as necessary */
    public void addEdge(int outID, int inID, Behavior b, double[] startXYT)
    {
        assert (nodes.containsKey(outID));
        assert (nodes.containsKey(inID));
        assert (outID != inID);

        Edge e = new Edge(b, startXYT);
        edges.put(e.id, e);

        Node out = nodes.get(outID);
        Node in = nodes.get(inID);

        // Does this edge exist? Check for outbound edges ending at in
        // of the same type.
        Edge match = null;
        for (Integer key: out.edgesOut) {
            Edge edgeOut = edges.get(key);
            if (e.equals(edgeOut)) {
                match = edgeOut;
                break;
            }
        }

        if (match != null) {
            match.startXYTs.add(startXYT);
            match.b.xyts.addAll(e.b.xyts);
            match.b.distances.addAll(e.b.distances);
        } else {
            out.edgesOut.add(e.id);
            in.edgesIn.add(e.id);
            if (!in.in2out.containsKey(e.id)) {
                in.in2out.put(e.id, new HashSet<Integer>());
            }
            match = e;
        }

        // Check to see if this edge chains with others by comparing XYTs. There
        // is, by necessity, some tolerance here, but make it quite small.
        // First, look through all incoming edges @ outID and see if they
        // overlap with and of match.xyts. Then, look at all outgoing edges @
        // inID and likewise look for overlap. We only need to consider the
        // most recently proposed xyts in behavior!
        for (double[] xyt: match.b.xyts) {
            for (Integer key: in.edgesOut) {
                Edge edgeOut = edges.get(key);
                for (double[] outXYT: edgeOut.startXYTs) {
                    if (xytEquals(xyt, outXYT)) {
                        in.in2out.get(match.id).add(edgeOut.id);
                    }
                }
            }
        }

        for (double[] xyt: match.startXYTs) {
            for (Integer key: out.edgesIn) {
                Edge edgeIn = edges.get(key);
                for (double[] inXYT: edgeIn.b.xyts) {
                    if (xytEquals(xyt, inXYT)) {
                        out.in2out.get(edgeIn.id).add(match.id);
                    }
                }
            }
        }
    }

    // XXX Eventually, want to change "startTag" to a specific
    // goal or orientation to match against graph structure.
    public ArrayList<Behavior> navigate(int startTag, int endTag, VisWorld vw)
    {
        // Perform a Dijkstra search through the graph. MOAR NODES
        DijkstraNode dn = new DijkstraNode(startTag);

        HashSet<DijkstraNode> closedList = new HashSet<DijkstraNode>();
        PriorityQueue<DijkstraNode> queue =
            new PriorityQueue<DijkstraNode>(10, new DijkstraComparator());
        queue.add(dn);

        VisWorld.Buffer vb = vw.getBuffer("debug-graph-nav");
        while (queue.size() > 0) {
            dn = queue.poll();

            // If this node reaches our goal, return a plan
            Edge currEdge = dn.getEdge();
            Node currNode = dn.getNode();
            if (currNode != null && currNode.id == endTag)
                return planHelper(dn); // XXX

            // Pass over previously visited search nodes
            if (closedList.contains(dn))
                continue;
            closedList.add(dn);

            // Generate children
            Set<Integer> validEdgesOut;
            if (currEdge == null)
                validEdgesOut = currNode.edgesOut;
            else
                validEdgesOut = currNode.in2out.get(currEdge.id);
            for (Integer edgeID: validEdgesOut) {
                queue.add(dn.addChild(edgeID));

                Edge e = edges.get(edgeID);
                vb.addBack(new VisChain(LinAlg.xytToMatrix(e.startXYTs.get(0)),
                                        new VzBox(new VzMesh.Style(Color.red))));
            }
            vb.swap();
            TimeUtil.sleep(1000);
        }
        vb.swap();

        System.err.println("ERR: Could not find path between tags");
        return null; // Failure
    }

    private ArrayList<Behavior> planHelper(DijkstraNode dn)
    {
        ArrayList<Behavior> plan = new ArrayList<Behavior>();
        while (dn.parent.parent != null) {
            Edge edge = dn.getEdge();
            plan.add(edge.b);
            dn = dn.parent;
        }
        plan.add(new Behavior(dn.getEdge().startXYTs.get(0), 0, null, null));

        Collections.reverse(plan);
        return plan;
    }

    /** Determine whether or not the graph is fully reachable. */
    public boolean isFullyReachable()
    {
        // XXX Highly ineffecient implementation to come?
        return false;
    }


}
