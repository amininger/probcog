package probcog.util;

import java.util.ArrayList;
import java.util.Formatter;

/** A generic tree structure supporting arbitrary numbers of branches.
 *  Generic type T represents the content of nodes of the tree.
 */
public class Tree<T>
{
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

        public Node(T data)
        {
            this(data, null);
        }

        public Node(T data, Node<T> parent)
        {
            this.data = data;
            this.parent = parent;
        }

        public void addChild(T data)
        {
            children.add(new Node(data, this));
        }

        public String toString()
        {
            Formatter f = new Formatter();
            f.format("data: [%s], %d children\n", data, children.size());
            return f.toString();
        }
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
        if (node.children.size() < 1)
            return;

        for (Node<T> n: node.children)
            traversalHelper(n, nodes);

        nodes.add(node);
    }
}
