package probcog.commands;

import java.util.*;

/** Encodes all control laws and associated termination conditions that can
 *  transition a robot from one location to another.
 **/
public class CommandEdge
{
    ArrayList<Edge> edges = new ArrayList<Edge>();
    public static class Edge
    {
        public String law;
        public HashMap<String, TypedValue> lawParams = new HashMap<String, TypedValue>();

        public String term;
        public HashMap<String, TypedValue> termParams = new HashMap<String, TypedValue>();

        public Edge(String law, String term)
        {
            this.law = law;
            this.term = term;
        }

        public void addLawParam(String name, TypedValue value)
        {
            lawParams.put(name, value);
        }

        public void addTermParam(String name, TypedValue value)
        {
            termParams.put(name, value);
        }
    }

    public CommandEdge()
    {
    }

    /** Add an edge value to this edge. Edges are potentially multi-valued
     *  as there may be many ways to transition from one point to another.
     **/
    public void addEdge(Edge e)
    {
        edges.add(e);
    }

    public ArrayList<Edge> getEdge()
    {
        return edges;
    }
}
