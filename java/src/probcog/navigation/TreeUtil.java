package probcog.navigation;

import java.awt.Color;
import java.io.*;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.commands.controls.*;
import probcog.util.*;
import probcog.util.Tree.Node;
import probcog.sim.*;

/** Used to interact with behavior trees in general ways.
 *  For example, to store/load the tree for future use.
 **/
public class TreeUtil
{
    static private class HistogramComparator implements Comparator<Integer>
    {
        HashMap<Integer, Integer> hits = null;
        public HistogramComparator(HashMap<Integer, Integer> hits)
        {
            this.hits = hits;
        }

        public int compare(Integer a, Integer b)
        {
            if (hits.get(a) > hits.get(b))
                return -1;
            else if (hits.get(b) > hits.get(a))
                return 1;
            return 0;
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof HistogramComparator))
                return false;
            HistogramComparator hc = (HistogramComparator)o;
            return hc == this;
        }
    }

    // Behaviors contain a LOT of data. How much of it do we
    // really need to save? Definitely the behaviors themselves.
    // Probably also theoretical start and end points, since we
    // use these to construct the routes for the "ideal" tree?
    // It would also be helpful to store probabilities, if
    // relevant, seeing as they play a direct role in scoring. Will
    // we want to keep those around, later?
    //
    // Basically, it's not clear what we will need to do later analysis,
    // so we should consider this before moving on...
    public void writeTree(Tree<Behavior> tree)
    {

    }

    public Tree<Behavior> readTree(String filename)
    {
        return null;
    }

    static public void renderTree(Tree<Behavior> tree, SimWorld sw, VisWorld.Buffer vb)
    {
        // Render the tree...
        java.util.List<Color> colors = Palette.diverging_brewer1.listAll();
        ArrayList<Tree.Node<Behavior> > nodes = tree.inOrderTraversal();
        //VisWorld.Buffer vb = vw.getBuffer("spanning-tree");
        vb.setDrawOrder(-2000);
        for (Tree.Node<Behavior> node: nodes) {
            if (node.parent == null)
                continue;
            MonteCarloBot mcb = new MonteCarloBot(sw);
            int retryCount = 100;
            //do {
            //Behavior.XYTPair pair = node.parent.data.randomXYT();
            mcb.init(node.data.law,
                     node.data.test,
                     node.parent.data.theoreticalXYT,
                     0);
            mcb.simulate(true);
            if (!mcb.success()) {
                System.out.printf("%s %s @ %d [%f %f %f]\n",
                                  node.data.law,
                                  node.data.test,
                                  node.data.tagID,
                                  node.parent.data.theoreticalXYT[0],
                                  node.parent.data.theoreticalXYT[1],
                                  node.parent.data.theoreticalXYT[2]);
                vb.addBack(new VisLighting(false, mcb.getVisObject(Color.red)));
            }
            int k0 = ((node.depth-1)/colors.size())%2;
            int k1 = (node.depth-1)%colors.size();
            int cidx = k0*(colors.size()-1) + (k0 == 0 ? 1:-1)*(k1);
            Color c = colors.get(cidx);
            c = colors.get((int)(node.data.prob * (colors.size()-1)));
            vb.addBack(new VisLighting(false, mcb.getVisObject(c)));
        }
        vb.swap();
    }

    static public HashMap<Integer, Tree<Behavior> > makeTrees(SimWorld sw,
                                                              GridMap gm,
                                                              VisWorld vw)
    {
        return makeTrees(sw, gm, vw, Long.MAX_VALUE);
    }

    /** Build all of the trees for the landmarks in the given world */
    static public HashMap<Integer, Tree<Behavior> > makeTrees(SimWorld sw,
                                                              GridMap gm,
                                                              VisWorld vw,
                                                              long timeout_ms)
    {
        MonteCarloPlanner mcp = new MonteCarloPlanner(sw, gm, null);
        HashMap<Integer, Tree<Behavior> > trees = new HashMap<Integer, Tree<Behavior> >();

        for (SimObject so: sw.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            SimAprilTag tag = (SimAprilTag)so;
            System.out.println("Building tree for tag "+tag.getID());
            Tree<Behavior> tree = mcp.buildSpanningTree(tag.getID(), timeout_ms);
            System.out.println("Done! Build tree size "+tree.size());
            trees.put(tag.getID(), tree);

            if (vw != null) {
                renderTree(tree, sw, vw.getBuffer("spanning-tree-"+tag.getID()));
            }
        }

        return trees;
    }

    /** Give a set of trees (presumably for one world), build a histogram of
     *  behavioral outcomes referencing the appropriate behaviors.
     *
     *  Returns a mapping of tag to hits
     **/
    static public HashMap<Integer, Integer> hist(HashMap<Integer, Tree<Behavior> > trees)
    {
        // Group by ending condition, then by control law. Want to be able to
        // sort on the number of "same control law" conditions we encounter.
        // There are two potential ways we'll end up being interested in this.
        // 1) Trajectories with similar end states, i.e. XYT.
        //  a) This might just mean basically identical...
        //  b) ...or it might mean close enough to follow the same result funnels
        // 2) Trajectories that end at the same location via the same behavior
        //  a) These seem less useful for a simplifying structure

        // First pass: count hits at each landmark
        HashMap<Integer, Integer> hits = new HashMap<Integer, Integer>();
        for (Integer key: trees.keySet()) {
            Tree<Behavior> tree = trees.get(key);
            ArrayList<Node<Behavior> > nodes = tree.inOrderTraversal();
            for (Node<Behavior> node: nodes) {
                if (!(node.data.law instanceof FollowWall))
                    continue;
                if (node.data.tagID < 0)
                    continue;
                if (!hits.containsKey(node.data.tagID))
                    hits.put(node.data.tagID, 0);
                hits.put(node.data.tagID, hits.get(node.data.tagID)+node.children.size());
            }
        }

        // End Analysis
        System.out.println();
        for (Integer key: hits.keySet()) {
            System.out.printf("Tag %d: %d\n", key, hits.get(key));
        }

        return hits;
    }

    /** Given a set of trees (presumably from one world), build a graph
     *  which summarizes the structure. Must ensure that every node is
     *  reachable from all others!
     *
     *  Unsure about whether to be completionist or merely minimalist. For
     *  first pass, try minimalist.
     **/
    static public BehaviorGraph compress(HashMap<Integer, Tree<Behavior> > trees)
    {
        HashMap<Integer, Integer> hits = hist(trees);
        ArrayList<Integer> sortedIDs = new ArrayList<Integer>(hits.keySet());
        Collections.sort(sortedIDs, new HistogramComparator(hits));

        // Initialize graph. Add nodes first, leaving them unconnected.
        // NOTE: An alternative strategy involves multiple nodes per tag ID.
        // This seems to be an easier way to build the structure
        BehaviorGraph graph = new BehaviorGraph();
        for (Integer id: sortedIDs)
            graph.addNode(id);

        // XXX TODO: Experiment with adding more yaws to child generation
        // Greedily add edges to the the graph by pulling them from the trees in
        // histogram order. Note: We should also be adding edges from the
        // beginning of the tree...children of root for that tree
        // STOP when the graph is fully connected.
        // NOTE: Must handle both "finishing" steps and actual movement around!
        // We can handle this by ignoring drive-to-tag steps when constructing
        // the graph and always adding these to the ends of plans.
        int idCount = 0;
        for (Integer id: sortedIDs) {
            idCount++;
            for (Integer key: trees.keySet()) {
                Tree<Behavior> tree = trees.get(key);
                // XXX Known inefficiencies! Could cache this somehow.
                ArrayList<Node<Behavior> > nodes = tree.inOrderTraversal();
                for (Node<Behavior> node: nodes) {
                    if (node.data.law instanceof DriveTowardsTag)
                        continue;
                    if (node.data.tagID != id)
                        continue;

                    for (Node<Behavior> child: node.children) {
                        if (child.data.law instanceof DriveTowardsTag)
                            continue;
                        if (child.data.tagID < 0)
                            continue;

                        // Add the edge. This either means creating the edge
                        // from scratch if it did not previously exist OR adding
                        // an XYT to an existing edge of the same type. Hopefully,
                        // that gives us adequate diversity to match against when
                        // looking up where to latch on to, later!
                        //
                        // XXX Is there any benefit to avoiding adding edges
                        // to things that are already connected by some other
                        // route?
                        graph.addEdge(id, child.data.tagID, child.data, child.parent.data.theoreticalXYT);
                    }

                    // There should only be one such edge. Break!
                    break;
                }

                // Additionally, add edges coming out of appropriate tree root
                if (key.equals(id)) {
                    for (Node<Behavior> child: tree.root.children) {
                        if (child.data.law instanceof DriveTowardsTag)
                            continue;
                        if (child.data.tagID < 0)
                            continue;

                        // Add the edge. This either means creating the edge
                        // from scratch if it did not previously exist OR adding
                        // an XYT to an existing edge of the same type. Hopefully,
                        // that gives us adequate diversity to match against when
                        // looking up where to latch on to, later!
                        //
                        // XXX Is there any benefit to avoiding adding edges
                        // to things that are already connected by some other
                        // route?
                        graph.addEdge(id, child.data.tagID, child.data, child.parent.data.theoreticalXYT);
                    }
                }
            }

            // We are done early if the graph is fully reachable
            if (graph.isFullyReachable())
                break;
        }

        // XXX I would expect to need all of these, since outgoing "bad" edges
        // may only exist in very particular cases. In other words, we are just
        // compressing the structure of many trees into one graph, which we
        // will need to search AGAIN.
        System.out.printf("Graph constructed using %d of %d landmarks.\n", idCount, sortedIDs.size());
        return graph;
    }
}
