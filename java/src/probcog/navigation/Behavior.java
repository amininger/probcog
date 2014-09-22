package probcog.navigation;

import java.awt.Color;
import java.util.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;

// Used to represent a law-test pair, along with some state for distributions
// of states associated with executing this pair.
public class Behavior
{
    Random r = new Random();

    // Control state
    public ControlLaw law;
    public ConditionTest test;
    //public FollowWall law;                  // Law to follow
    //public ClassificationCounterTest test;  // Test to check against

    // Bookkeeping for evaluating search.
    private double behaviorScore = Double.MAX_VALUE;
    public int ageOfLastObservation;
    public ArrayList<double[]> xyts = new ArrayList<double[]>();
    public ArrayList<Double> distances = new ArrayList<Double>();
    public class XYTPair
    {
        public double[] xyt;
        public double dist;

        public XYTPair(double[] xyt, double dist)
        {
            this.xyt = xyt;
            this.dist = dist;
        }
    }

    XYTStats stats = null;
    static private class XYTStats
    {
        public double[] mean;
        public double[][] cov;
    }

    ArrayList<Cluster> clusters = new ArrayList<Cluster>();
    static public class Cluster
    {
        // Computed statistics about xyts
        public ArrayList<double[]> xyts = new ArrayList<double[]>();
        public XYTStats stats = null;

        public void addXYT(double[] xyt)
        {
            xyts.add(xyt);
            stats = null;
        }

        public void evaluate()
        {
            if (stats != null)
                return;
            stats = computeStats(xyts);
        }

        public double[] getMean()
        {
            evaluate();
            return stats.mean;
        }

        public double[][] getVar()
        {
            evaluate();
            return stats.cov;
        }

        public int size()
        {
            return xyts.size();
        }
    }

    static public class ClusterSizeComparator implements Comparator<Cluster>
    {
        public int compare(Cluster a, Cluster b)
        {
            int sa = a.xyts.size();
            int sb = b.xyts.size();

            if (sa > sb)
                return -1;
            else if (sa < sb)
                return -1;
            return 0;
        }
    }

    //public Behavior(double[] xyt, double distance, FollowWall law, ClassificationCounterTest test)
    public Behavior(double[] xyt, double distance, ControlLaw law, ConditionTest test)
    {
        this(0, xyt, distance, law, test);
    }

    //public Behavior(int age, double[] xyt, double distance, FollowWall law, ClassificationCounterTest test)
    public Behavior(int age, double[] xyt, double distance, ControlLaw law, ConditionTest test)
    {
        xyts.add(xyt);
        distances.add(distance);
        ageOfLastObservation = age;
        this.law = law;
        this.test = test;
    }

    //public Behavior(ArrayList<double[]> xyts, ArrayList<Double> distances, FollowWall law, ClassificationCounterTest test)
    public Behavior(ArrayList<double[]> xyts, ArrayList<Double> distances, ControlLaw law, ConditionTest test)
    {
        assert (xyts.size() == distances.size());

        ageOfLastObservation = 0;
        this.xyts = xyts;
        this.law = law;
        this.test = test;
        this.distances = distances;
    }

    // Get the best possible score we could still achieve with this chain of
    // behaviors. This is some combination of our distribution of XYTS along with
    // the distance we likely traveled to arrive at that distribution. We prefer
    // to travel less and arrive more reliably. Note: lower scores are BETTER
    //
    // XXX TODO: Balancing our needs in an understandable way, here
    //
    // The grid map and corresponding wavefront are used to evaluate best-case
    // distance to the goal from our current location.
    public double getBestScore(GridMap gm, float[] wavefront, int numSamples, int depth)
    {
        if (behaviorScore < Double.MAX_VALUE)
            return behaviorScore;

        double meanDistance = 0;
        for (int i = 0; i < xyts.size(); i++) {
            double dist = distances.get(i);
            double[] xyt = xyts.get(i);
            int ix = (int)(Math.floor((xyt[0]-gm.x0)/gm.metersPerPixel));
            int iy = (int)(Math.floor((xyt[1]-gm.y0)/gm.metersPerPixel));
            double wfdist = (double)wavefront[iy*gm.width + ix];

            meanDistance += .95*dist + wfdist;
        }

        meanDistance /= xyts.size();

        // We (incorrectly) assert that we will never recover from being spread
        // out, much like in DART. This lets us bound our search earlier.
        getClusters();
        double maxClusterPct = (double)clusters.get(0).size()/(double)numSamples;
        behaviorScore = meanDistance - 50.0*(maxClusterPct);    // Not perfect, but interesting
        return behaviorScore;
    }

    public double getPctNearGoal(GridMap gm, float[] wavefront, int numSamples)
    {
        if (numSamples < 1)
            return 0;

        double GOAL_THRESH = 4.0;
        int count = 0;
        for (int i = 0; i < xyts.size(); i++) {
            double[] xyt = xyts.get(i);
            int ix = (int)(Math.floor((xyt[0]-gm.x0)/gm.metersPerPixel));
            int iy = (int)(Math.floor((xyt[1]-gm.y0)/gm.metersPerPixel));
            double wfdist = (double)wavefront[iy*gm.width + ix];
            if (wfdist < GOAL_THRESH)
                count++;
        }

        return (double)count/(double)numSamples;
    }

    public double getMeanDistTraveled()
    {
        if (distances.size() < 1)
            return 0;

        double mean = 0;
        for (int i = 0; i < distances.size(); i++) {
            mean += distances.get(i);
        }

        return mean / distances.size();
    }

    public double getMeanDistToGoal(GridMap gm, float[] wavefront)
    {
        if (xyts.size() < 1)
            return Double.MAX_VALUE;

        double mean = 0;
        for (int i = 0; i < xyts.size(); i++) {
            double[] xyt = xyts.get(i);
            int ix = (int)(Math.floor((xyt[0]-gm.x0)/gm.metersPerPixel));
            int iy = (int)(Math.floor((xyt[1]-gm.y0)/gm.metersPerPixel));
            double wfdist = (double)wavefront[iy*gm.width + ix];
            mean += wfdist;
        }

        return mean /= xyts.size();
    }

    public void addObservation(double[] xyt, double dist)
    {
        xyts.add(xyt);
        distances.add(dist);
        stats = null;
    }

    public XYTPair getXYT()
    {
        assert (xyts.size() > 0);
        return new XYTPair(xyts.get(0), distances.get(0));
    }

    // Uniformly sample an XYT from the samples
    public XYTPair randomXYT()
    {
        assert (xyts.size() > 0);
        int idx = r.nextInt(xyts.size());
        return new XYTPair(xyts.get(idx), distances.get(idx));
    }

    private void evaluate()
    {
        if (stats != null)
            return;
        stats = computeStats(xyts);

        // Clustering. Quite naive
        clusters.clear();
        for (double[] xyt: xyts) {
            double bestDist = Double.MAX_VALUE;
            Cluster bestCluster = null;
            for (Cluster c: clusters) {
                double dist = LinAlg.distance(xyt, c.getMean(), 2);
                if (dist < bestDist) {
                    bestDist = dist;
                    bestCluster = c;
                }
            }

            // XXX Magical clustering parameter
            if (bestDist < 0.5) {
                bestCluster.addXYT(xyt);
            } else {
                Cluster c = new Cluster();
                c.addXYT(xyt);
                clusters.add(c);
            }
        }

        // Evaluate clusters
        int numxyts = 0;
        for (Cluster c: clusters)
            numxyts += c.size();
        assert (numxyts == xyts.size());

        Collections.sort(clusters, new ClusterSizeComparator());
    }

    public double[] getMean()
    {
        evaluate();
        return stats.mean;
    }

    public double[][] getVar()
    {
        evaluate();
        return stats.cov;
    }

    public ArrayList<Cluster> getClusters()
    {
        evaluate();
        assert (clusters.size() > 0);
        return clusters;
    }


    public VisObject getVisObject()
    {
        VisVertexData vvd = new VisVertexData();
        for (double[] xyt: xyts)
            vvd.add(xyt, 1, 2); // Not very efficient
        return new VzPoints(vvd, new VzPoints.Style(Color.red, 5));
    }

    // === Utility ===========================================================
    static private XYTStats computeStats(ArrayList<double[]> xyts)
    {
        XYTStats stats = new XYTStats();
        stats.mean = new double[3];
        stats.cov = new double[3][3];

        if (xyts.size() < 1)
            return stats;

        for (int i = 0; i < xyts.size(); i++) {
            double[] xyt = xyts.get(i);
            LinAlg.plusEquals(stats.mean, xyt);
            for (int c = 0; c < 3; c++) {
                for (int r = 0; r < 3; r++) {
                    stats.cov[r][c] += xyt[r]*xyt[c];
                }
            }
        }

        stats.mean = LinAlg.scale(stats.mean, 1.0/xyts.size());
        for (int c = 0; c < 3; c++) {
            for (int r = 0; r < 3; r++) {
                stats.cov[r][c] /= xyts.size();
                stats.cov[r][c] -= stats.mean[r]*stats.mean[c];
            }
        }

        return stats;
    }

    public String toString()
    {
        return String.format("%s until %s\n", law.toString(), test.toString());
    }

    // XXX These only exist to support MonteCarloBot's hash table for class counting.
    public int hashCode()
    {
        assert (test instanceof ClassificationCounterTest);
        ClassificationCounterTest cct = (ClassificationCounterTest)test;
        return new Integer(cct.getCount()).hashCode() ^ cct.getClassType().hashCode();
    }

    public boolean equals(Object o)
    {
        if (o == null)
            return false;
        if (!(o instanceof Behavior))
            return false;
        Behavior b = (Behavior)o;

        assert (test instanceof ClassificationCounterTest);
        ClassificationCounterTest cct = (ClassificationCounterTest)test;
        ClassificationCounterTest bcct = (ClassificationCounterTest)b.test;
        return cct.getCount() == bcct.getCount() && cct.getClassType().equals(bcct.getClassType());
    }
}
