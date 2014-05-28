package probcog.commands;

import java.util.*;

import april.jmat.*;

import probcog.util.*;

public class GraphUtil
{
    public static ArrayList<CommandEdge.Edge> bestPath(SimpleGraph<CommandNode, CommandEdge> g,
                                                      double[] start,
                                                      double[] fin)
    {
        // Find start and end nodes
        SimpleGraphNode s, f;
        s = f = null;
        double sDist, fDist;
        sDist = fDist = Double.POSITIVE_INFINITY;
        for (SimpleGraphNode n: g.getNodes()) {
            CommandNode val = g.getNodeValue(n);
            if (s == null)
                s = n;
            if (f == null)
                f = n;
            if (LinAlg.distance(start, val.getXY()) < sDist) {
                sDist = LinAlg.distance(start, val.getXY());
                s = n;
            }
            if (LinAlg.distance(fin, val.getXY()) < fDist) {
                fDist = LinAlg.distance(fin, val.getXY());
                f = n;
            }
        }

        //System.out.printf("[%f %f] [%f %f]\n",
        //        g.getNodeValue(s).getXY()[0],
        //        g.getNodeValue(s).getXY()[1],
        //        g.getNodeValue(f).getXY()[0],
        //        g.getNodeValue(f).getXY()[1]);

        assert (s != null && f != null);

        // Dumb BFS
        Node root = new Node(null, s, null);
        ArrayList<Node> front = new ArrayList<Node>();
        ArrayList<Node> leaves = new ArrayList<Node>();
        front.add(root);
        while (front.size() > 0) {
            ArrayList<Node> next = new ArrayList<Node>();
            for (Node n: front) {
                Set<SimpleGraphNode> neighbors = g.neighbors(n.node);
                if (neighbors.size() == 0) {
                    leaves.add(n);
                    continue;
                }
                for (SimpleGraphNode bnode: neighbors) {
                    CommandEdge ce = g.getEdgeValue(n.node, bnode);
                    for (CommandEdge.Edge edge: ce.getEdges()) {
                        next.add(n.makeChild(bnode, edge));
                    }
                }
            }

            front = next;
        }

        // Find best path
        double bestValue = -1;
        Node bestNode = null;
        //int cnt = 1;
        for (Node n: leaves) {
            if (!n.node.equals(f)) {
            //    System.out.printf("skip: %d\n", cnt++);
                continue;
            }
            double value = 1.0;
            Node curr = n;
            while (curr.parent != null) {
                value *= 0.9999*curr.value.probability;  // Long run, log...
                curr = curr.parent;
            }
            if (value > bestValue) {
                bestValue = value;
                bestNode = n;
            }
        }

        // Return plan
        ArrayList<CommandEdge.Edge> path = new ArrayList<CommandEdge.Edge>();
        while (bestNode.parent != null) {
            path.add(bestNode.value);
            bestNode = bestNode.parent;
        }

        Collections.reverse(path);
        return path;
    }

    static class Node
    {
        public Node parent;
        public SimpleGraphNode node;
        public CommandEdge.Edge value;

        public Node(Node p, SimpleGraphNode n, CommandEdge.Edge v)
        {
            parent = p;
            node = n;
            value = v;
        }

        public Node makeChild(SimpleGraphNode n, CommandEdge.Edge v)
        {
            return new Node(this, n ,v);
        }
    }
}
