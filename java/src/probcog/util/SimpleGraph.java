package probcog.util;

/** A generic structure for representing both directed and undirected graphs.
 *
 *  T == Node (potentially with value) , U == Edge Value
 **/
public class SimpleGraph<T extends SimpleGraphNode, U>
{
    // Nodes
    HashSet<T> nodes = new HashMap<T>();

    // Directional edges (undirected graphs just have edges in both directions.
    // This implementation is based on the assumption that our graph is sparse.
    HashMap<T, HashMap<T, U> > edges = new HashMap<T, HashMap<T, U> >();

    // === Graph management ===
    /** Add a node to the graph */
    public addNode(T t)
    {
        // XXX
    }

    /** Put an undirected edge between nodes a and b with value v. */
    public void connectAll(T a, T b, U v)
    {
        connect(a, b, v);
        connect(b, a, v);
    }

    /** Put a directed edge between nodes a and b with value v. */
    public void connect(T a, T b, U v)
    {
        // XXX
    }

    public void disconnectAll(T a, T b)
    {
        disconnect(a, b);
        disconnect(b, a);
    }

    public void disconnect(T a, T b)
    {
        // XXX
    }

    public void remove(T n)
    {
        // XXX
    }

    /** Reset the graph */
    public void clear()
    {
        nodes.clear();
        edges.clear();
    }

