package probcog.navigation;

import java.util.*;

import april.jmat.*;

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
    // XXX Try to write the algorithm, first, and let this inform the interface

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

    private boolean xytEquals(double[] xyt0, double[] xyt1)
    {
        double dx = Math.abs(xyt0[0] - xyt1[0]);
        double dy = Math.abs(xyt0[1] - xyt1[1]);
        double dt = Math.abs(MathUtil.mod2pi(xyt0[2] - xyt1[2]));

        return dx < 0.2 && dy < 0.2 && dt < Math.toRadians(5);  // XXX
    }

    /** Add a node to the graph */
    public void addNode(int tagID)
    {
        nodes.put(tagID, new Node(tagID)); // XXX Do we care about XY location?
    }

    /** Add an edge to the graph, or modify an existing edge as necessary */
    public void addEdge(int outID, int inID, Behavior b)
    {
        assert (nodes.containsKey(outID));
        assert (nodes.containsKey(inID));
        assert (outID != inID);

        Edge e = new Edge(b);
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
        for (double[] xyt: b.xyts) {
            for (Integer key: out.edgesIn) {
                Edge edgeIn = edges.get(key);
                for (double[] inXYT: edgeIn.b.xyts) {
                    if (xytEquals(xyt, inXYT)) {
                        out.in2out.get(edgeIn.id).add(match.id);
                    }
                }
            }

            for (Integer key: in.edgesOut) {
                Edge edgeOut = edges.get(key);
                for (double[] outXYT: edgeOut.b.xyts) {
                    if (xytEquals(xyt, outXYT)) {
                        in.in2out.get(match.id).add(edgeOut.id);
                    }
                }
            }
        }
    }

    public boolean isFullyConnected()
    {
        System.err.println("Not yet implemented");
        return false;
    }


}
