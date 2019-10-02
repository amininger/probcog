package soargroup.mobilesim.util;

import java.util.ArrayList;
import java.util.Formatter;

/** A generic tree structure supporting arbitrary numbers of branches.
 *  Generic type T represents the content of nodes of the tree.
 */
public class Tree<T>
{
    static int nodeID = 0;
    public Node<T> root;

    public Tree(T rootData)
    {
        root = new Node(rootData, null);
    }

    public static class Node<T>
    {
        public T data;
        public Node<T> parent;
        public ArrayList<Node<T> > children = new ArrayList<Node<T> >();
        public int depth;

        private int id;

        public Node(T data)
        {
            this(data, null);
        }

        public Node(T data, Node<T> parent)
        {
            this.data = data;
            this.parent = parent;
            this.depth = 0;
            this.id = nodeID++;
        }

        public Node<T> addChild(T data)
        {
            Node<T> n = new Node(data, this);
            n.depth = depth+1;
            children.add(n);
            return n;
        }

        public void removeChild(Node<T> child)
        {
            for (int i = 0; i < children.size(); i++) {
                Node<T> n = children.get(i);
                if (n.id == child.id) {
                    children.remove(n);
                    child.parent = null;
                    return;
                }
            }
        }

        public String toString()
        {
            Formatter f = new Formatter();
            f.format("data: [%s], %d children\n", data, children.size());
            return f.toString();
        }

        public int size()
        {
            int s = 1;
            for (Node<T> child: children) {
                s += child.size();
            }
            return s;
        }
    }

    public int size()
    {
        return root.size();
    }

    /** Return a list of all of the leaves in the tree */
    public ArrayList<Node<T> > getLeaves()
    {
        ArrayList<Node<T> > leaves = new ArrayList<Node<T> >();
        leafHelper(root, leaves);
        return leaves;
    }

    private void leafHelper(Node<T> node, ArrayList<Node<T> > leaves)
    {
        if (node.children.size() < 1) {
            leaves.add(node);
            return;
        }

        for (Node<T> child: node.children)
            leafHelper(child, leaves);
    }

    /** Return a list of the nodes of this tree for in-order traversal */
    public ArrayList<Node<T> > inOrderTraversal()
    {
        ArrayList<Node<T> > nodes = new ArrayList<Node<T> >();
        traversalHelper(root, nodes);
        return nodes;
    }

    private void traversalHelper(Node<T> node, ArrayList<Node<T> > nodes)
    {
        for (Node<T> n: node.children)
            traversalHelper(n, nodes);

        nodes.add(node);
    }
}
