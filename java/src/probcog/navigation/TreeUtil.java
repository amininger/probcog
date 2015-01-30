package probcog.navigation;

import java.awt.Color;
import java.io.*;
import java.util.*;

import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.util.*;
import probcog.sim.*;

/** Used to interact with behavior trees in general ways.
 *  For example, to store/load the tree for future use.
 **/
public class TreeUtil
{
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

    static public void renderTree(Tree<Behavior> tree, SimWorld sw, VisWorld vw)
    {
        // Render the tree...
        java.util.List<Color> colors = Palette.diverging_brewer.listAll();
        ArrayList<Tree.Node<Behavior> > nodes = tree.inOrderTraversal();
        VisWorld.Buffer vb = vw.getBuffer("spanning-tree");
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
                System.out.printf("%s %s @ [%f %f %f]\n",
                                  node.data.law,
                                  node.data.test,
                                  node.parent.data.theoreticalXYT[0],
                                  node.parent.data.theoreticalXYT[1],
                                  node.parent.data.theoreticalXYT[2]);
            }
            //    retryCount--;
            //} while (!mcb.success() && retryCount > 0);
            int k0 = ((node.depth-1)/colors.size())%2;
            int k1 = (node.depth-1)%colors.size();
            int cidx = k0*(colors.size()-1) + (k0 == 0 ? 1:-1)*(k1);
            Color c = colors.get(cidx);
            vb.addBack(mcb.getVisObject(c));
        }
        vb.swap();
    }

    static public HashMap<Integer, Tree<Behavior> > makeTrees(SimWorld sw, GridMap gm, VisWorld vw)
    {
        MonteCarloPlanner mcp = new MonteCarloPlanner(sw, gm, null);
        HashMap<Integer, Tree<Behavior> > trees = new HashMap<Integer, Tree<Behavior> >();

        for (SimObject so: sw.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            SimAprilTag tag = (SimAprilTag)so;
            System.out.println("Building tree for tag "+tag.getID());
            Tree<Behavior> tree = mcp.buildSpanningTree(tag.getID());
            System.out.println("Done! Build tree size "+tree.size());
            trees.put(tag.getID(), tree);

            if (vw != null) {
                renderTree(tree, sw, vw);
            }
        }

        return trees;
    }
}
