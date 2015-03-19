package probcog.util;

import java.util.*;

import april.util.*;

/** Multi-tiered timing utility class for analyzing code performance. */
public class Stopwatch
{
    HashMap<String, Tic> tics = new HashMap<String, Tic>();

    TimeNode root;
    TimeNode leaf;

    static private class TimeNode
    {
        public double time;
        public String name;
        public TimeNode parent;
        public HashMap<String, TimeNode> children =
            new HashMap<String, TimeNode>();
        //public ArrayList<TimeNode> children = new ArrayList<TimeNode>();

        public TimeNode(TimeNode parent, String name)
        {
            this.parent = parent;
            this.name = name;
        }

        public TimeNode addChild(String name)
        {
            if (!children.containsKey(name)) {
                TimeNode child = new TimeNode(this, name);
                children.put(name, child);
            }
            return children.get(name);
        }

        public void addTime(double time)
        {
            this.time += time;
        }
    }

    public Stopwatch()
    {
        root = new TimeNode(null, null);
        leaf = root;
    }

    /** Please use unique names */
    public void start(String name)
    {
        if (!tics.containsKey(name)) {
            tics.put(name, new Tic());
        }
        tics.get(name).tic();
        leaf = leaf.addChild(name);
    }

    /** Stops most recently started Tic */
    public void stop()
    {
        if (leaf.parent == null) {
            System.err.println("ERR: Cannot stop null watch");
            return;
        }
        String name = leaf.name;
        assert (tics.containsKey(name));
        leaf.addTime(tics.get(name).toc());
        leaf = leaf.parent;
    }

    public void print()
    {
        printHelper(root, -1);
    }

    private void printHelper(TimeNode node, int d)
    {
        if (d >= 0) {
            String tabs = "";
            for (int i = 0; i < d; i++)
                tabs += " ";
            System.out.printf("%s%s: %f\n", tabs, node.name, node.time);
        }

        for (TimeNode child: node.children.values())
            printHelper(child, d+1);
    }

    static public void main(String[] args)
    {
        Stopwatch sw = new Stopwatch();
        sw.start("Top");
        for (int i = 0; i < 3; i++) {
            sw.start("L1."+i);
            for (int j = 0; j < 5; j++) {
                sw.start("L2."+j);
                TimeUtil.sleep(50);
                sw.stop();
            }
            sw.stop();
        }
        sw.stop();

        sw.print();
    }
}
