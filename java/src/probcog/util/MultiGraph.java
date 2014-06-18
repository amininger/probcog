package probcog.util;

import java.io.*;
import java.util.*;

/** Multi-edged graph with node values T and edge values U */
public class MultiGraph<T extends Serializable, U extends Serializable> implements Serializable
{
    static int ID_COUNTER = 0;

    // Store all nodes of the graph
    Map<Integer, T> nodes = new HashMap<Integer, T>();

    // Store all edges of the graph
    Map<Integer, Map<Integer, Set<U> > > edges = new HashMap<Integer, Map<Integer, Set<U> > >();

    // === Graph Management Interface ===
    /** Add a new node to the graph with the given value.
     *
     *  @param t    Value to assign to the node (or null for nothing)
     *
     *  @return     The ID of the node created
     **/
    public int addNode(T t)
    {
        int key = ID_COUNTER++;
        nodes.put(key, t);
        return key;
    }

    /** Remove a node from the graph
     *
     *  @param n    ID of the node to be removed
     *
     *  @return True if node was successfully removed
     **/
    public boolean removeNode(int n)
    {
        if (!nodes.containsKey(n))
            return false;

        // Clean up edges and remove node
        for (Integer a: edges.keySet()) {
            if (edges.get(a).containsKey(n))
                edges.get(a).remove(n);
        }
        if (edges.containsKey(n))
            edges.remove(n);
        nodes.remove(n);
        return true;
    }

    /** Add a directed edge between two nodes with the given value.
     *
     *  @param a    The ID of the start node
     *  @param b    The ID of the end node
     *  @param u    The edge value (or null for nothing)
     *
     *  @return True if the edge was added. False if one of the nodes DNE.
     **/
    public boolean addEdge(int a, int b, U u)
    {
        if (!(nodes.containsKey(a) && nodes.containsKey(b)))
            return false;

        if (!edges.containsKey(a))
            edges.put(a, new HashMap<Integer, Set<U> >());
        Map<Integer, Set<U> > edgeSet = edges.get(a);
        if (!edgeSet.containsKey(b))
            edgeSet.put(b, new HashSet<U>());
        edgeSet.get(b).add(u);

        return true;
    }

    public Set<Integer> getNodes()
    {
        return nodes.keySet();
    }

    /**  Return the value of node n. Can return null! */
    public T getValue(int n)
    {
        return nodes.get(n);
    }

    public Set<U> getEdges(int a, int b)
    {
        if (!edges.containsKey(a))
            return null;
        if (!edges.get(a).containsKey(b))
            return null;
        return edges.get(a).get(b);
    }

    // XXX Remove edge? Some extra effort needed for multi-edges. Implement if
    // this becomes useful

    // ==================================

    // === Serialization Interface ======
    //
    // XXX None...we will save the graph externally
    //
    // ==================================

    // === Planning Interface ===========
    /** Get the neighbors of the given node
     *
     *  @param n    ID of the node to get neighbors from
     *
     *  @return A set of the neighboring node IDs
     **/
    public Set<Integer> neighbors(int n)
    {
        if (edges.containsKey(n))
            return edges.get(n).keySet();
        return new HashSet<Integer>();
    }

    // ==================================

}
