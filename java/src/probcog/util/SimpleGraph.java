package probcog.util;

import java.io.*;
import java.util.*;

import april.util.*;

// XXX Serialization not straightforward...e.g. threads
/** A generic structure for representing both directed and undirected graphs.
 *
 *  T == Node value , U == Edge value
 **/
public class SimpleGraph<T, U>
{
    // Node list
    HashMap<SimpleGraphNode, T> nodes = new HashMap<SimpleGraphNode, T>();

    // Directional edges (undirected graphs just have edges in both directions.
    // This implementation is based on the assumption that our graph is sparse
    // and will never have more than 1 edge between nodes. If you need multi-edge
    // support, you can write in multiple edge values in your implementation of
    // U, potentially.
    HashMap<SimpleGraphNode, HashMap<SimpleGraphNode, U> > edges = new HashMap<SimpleGraphNode, HashMap<SimpleGraphNode, U> >();

    // === Graph management ===
    /** Add a node to the graph */
    public SimpleGraphNode addNode()
    {
        SimpleGraphNode n = new SimpleGraphNode();
        nodes.put(n, null);
        return n;
    }

    /** Add a node and a value to the graph */
    public SimpleGraphNode addNode(T t)
    {
        SimpleGraphNode n = new SimpleGraphNode();
        nodes.put(n, t);
        return n;
    }

    /** Put an undirected edge between nodes a and b*/
    public void connectAll(SimpleGraphNode a, SimpleGraphNode b)
    {
        connect(a, b);
        connect(b, a);
    }

    /** Put a directed edge between nodes a and b*/
    public void connect(SimpleGraphNode a, SimpleGraphNode b)
    {
        if (!edges.containsKey(a))
            edges.put(a, new HashMap<SimpleGraphNode, U>());
        if (!edges.get(a).containsKey(b))
            edges.get(a).put(b, null);
    }

    /** Put an undirected edge between nodes a and b with value u. */
    public void connectAll(SimpleGraphNode a, SimpleGraphNode b, U u)
    {
        connect(a, b, u);
        connect(b, a, u);
    }

    /** Put a directed edge between nodes a and b with value u. */
    public void connect(SimpleGraphNode a, SimpleGraphNode b, U u)
    {
        if (!edges.containsKey(a))
            edges.put(a, new HashMap<SimpleGraphNode, U>());
        if (!edges.get(a).containsKey(b))
            edges.get(a).put(b, u);
    }

    /** Remove all edges between nodes a and b */
    public void disconnectAll(SimpleGraphNode a, SimpleGraphNode b)
    {
        disconnect(a, b);
        disconnect(b, a);
    }

    /** Remove any edges from node a to node b */
    public void disconnect(SimpleGraphNode a, SimpleGraphNode b)
    {
        if (edges.containsKey(a))
            edges.get(a).remove(b);
    }

    /** Remove a node and its edges from the graph */
    public void remove(SimpleGraphNode n)
    {
        ArrayList<SimpleGraphNode> list = new ArrayList<SimpleGraphNode>();

        // Clean up edge references
        for (SimpleGraphNode e: neighbors(n)) {
            list.add(e);
        }
        for (SimpleGraphNode e: list) {
            disconnectAll(n, e);
        }

        nodes.remove(n);
        edges.remove(n);
    }

    /** Reset the graph */
    public void clear()
    {
        nodes.clear();
        edges.clear();
    }

    // === General queries ===
    /** Return all of the nodes */
    public Set<SimpleGraphNode> getNodes()
    {
        return nodes.keySet();
    }

    /** Return all neighbors of n */
    public Set<SimpleGraphNode> neighbors(SimpleGraphNode n)
    {
        if (!edges.containsKey(n))
            return new HashSet<SimpleGraphNode>();
        return edges.get(n).keySet();
    }

    /** True if a has an edge to b */
    public boolean adjacent(SimpleGraphNode a, SimpleGraphNode b)
    {
        if (!edges.containsKey(a))
            return false;
        return edges.get(a).containsKey(b);
    }

    // === Value queries and interactions ===
    /** Return the node value for n if it exists */
    public T getNodeValue(SimpleGraphNode n)
    {
        if (n == null)
            return null;
        return nodes.get(n);
    }

    /** Return the edge value for a--b if it exists */
    public U getEdgeValue(SimpleGraphNode a, SimpleGraphNode b)
    {
        if (a == null || b == null)
            return null;
        if (!edges.containsKey(a))
            return null;
        return edges.get(a).get(b);
    }

    /** Set the value for a node, also implicitly adding that node if
     *  it did not exist.
     **/
    public void setNodeValue(SimpleGraphNode n, T t)
    {
        nodes.put(n, t);
    }

    /** Set the edge value for a--b and b--a. Also implicitly creates
     *  an undirected edge between the two
     **/
    public void setEdgeValueAll(SimpleGraphNode a, SimpleGraphNode b, U u)
    {
        setEdgeValue(a, b, u);
        setEdgeValue(b, a, u);
    }

    /** Set the edge value for a--b. Also implicitly
     *  adds the edge if it does not already exist.
     **/
    public void setEdgeValue(SimpleGraphNode a, SimpleGraphNode b, U u)
    {
        if (!edges.containsKey(a))
            edges.put(a, new HashMap<SimpleGraphNode, U>());
        edges.get(a).put(b, u);
    }

    // === Saving and loading ===
    /** Write the graph to filename */
    /*public void write(String filename) throws IOException
    {
        BinaryStructureWriter fout = new BinaryStructureWriter(new BufferedOutputStream(new FileOutputStream(filename)));

        // Write out nodes
        fout.writeInt(nodes.size());
        fout.blockBegin();
        for (SimpleGraphNode n: nodes.keySet()) {
            n.write(fout);
            T t = nodes.get(n);
            if (t == null) {
                fout.writeInt(0);
                continue;
            }
            fout.writeInt(1);
            t.write(fout);
        }
        fout.blockEnd();

        // Write out edges
        fout.writeInt(edges.size());
        fout.blockBegin();
        for (SimpleGraphNode a: edges.keySet()) {
            a.write(fout);
            if (edges.get(a) == null) {
                fout.writeInt(0);
                continue;
            }
            fout.write(edges.get(a).size());
            for (SimpleGraphNode b: edges.get(a).keySet()) {
                b.write(fout);
                U u = edges.get(a).get(b);
                if (u == null) {
                    fout.writeInt(0);
                    continue;
                }
                fout.writeInt(1);
                u.write(fout);
            }
        }
        fout.blockEnd();
    }*/

    /** Reconstruct the graph stored in filename */
    /*public void read(String filename) throws IOException
    {
        BinaryStructureReader fin = new BinaryStructureReader(new BufferedInputStream(new FileInputStream(filename)));

        // Ensure empty structures
        reset();

        // Read in nodes and node values
        int size = fout.readInt();
        fout.blockBegin();
        for (int i = 0; i < size; i++) {
            SimpleGraphNode n = new SimpleGraphNode();
            n.read(fin);
            T t = null;
            int k = fin.readInt();
            if (k > 0) {
                T t = new T()
            }
        }
        fout.blockEnd();
    }*/
}
