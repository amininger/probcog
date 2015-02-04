package probcog.navigation;

import java.util.*;

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
        Map<Integer, Integer> in2out = new HashMap<Integer, Integer>();

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
        }
    }

    public boolean isFullyConnected()
    {
        System.err.println("Not yet implemented");
        return false;
    }


}
