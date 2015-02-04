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
    // LAMBA should be selected such that you are willing to travel an extra
    // LAMBA/100 meters to gain a 1% improvement in arrival rate.
    // XXX CONFIG
    static final double LAMBDA = 1000.0;
    Random r = new Random();

    // Control state
    public ControlLaw law;
    public ConditionTest test;

    // Some internal state for enabling better tree/path generation.
    // The ID lets us know what landmark we're supposed to be at, when the
    // control law is finished executing, while
    // theoreticalXYT lets us know where we should have been when we saw it.
    // Likewise, theoreticalDistance tells us how far we think we've traveled
    // at that point
    public int tagID = -1;
    public double[] theoreticalXYT = new double[3];
    public double theoreticalDistance = 0;

    // The probability that this behavior will actually be executed correctly.
    // For counting-based behaviors, one would expect this to be our estimate of
    // the probability that we correctly identify every critical tag along the way
    public double prob = 1.0;           // Cumulative probability to date
    public double myprob = 1.0;         // Probability of executing THIS behavior correctly

    // Bookkeeping for evaluating search.
    private double behaviorScore = Double.MAX_VALUE;
    private double scoreSoFar = Double.MAX_VALUE;
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

    private Behavior()
    {
    }

    public Behavior(double[] xyt, double distance, ControlLaw law, ConditionTest test)
    {
        xyts.add(xyt);
        distances.add(distance);
        this.law = law;
        this.test = test;
        this.theoreticalXYT = LinAlg.copy(xyt);
        this.theoreticalDistance = distance;
    }

    public Behavior(ArrayList<double[]> xyts, ArrayList<Double> distances, ControlLaw law, ConditionTest test)
    {
        assert (xyts.size() == distances.size());

        this.xyts = xyts;
        this.law = law;
        this.test = test;
        this.distances = distances;
        if (xyts.size() > 0) {
            this.theoreticalXYT = LinAlg.copy(xyts.get(0));
            this.theoreticalDistance = distances.get(0);
        }
    }

    public Behavior copyBehavior()
    {
        Behavior b = new Behavior();
        for (double[] xyt: xyts)
            b.xyts.add(LinAlg.copy(xyt));
        for (Double dist: distances)
            b.distances.add(dist);
        b.test = test.copyCondition();
        b.law = law;
        b.tagID = tagID;
        b.theoreticalXYT = LinAlg.copy(theoreticalXYT);
        b.theoreticalDistance = theoreticalDistance;
        b.prob = prob;
        b.myprob = myprob;

        return b; // XXX
    }

    public double getScoreSoFar(int numSamples)
    {
        getClusters();
        double maxClusterPct = (double)clusters.get(0).size()/(double)numSamples;
        return getScoreSoFar(maxClusterPct);
    }

    public double getScoreSoFar(double pct)
    {
        if (scoreSoFar < Double.MAX_VALUE)
            return scoreSoFar;

        double meanDistance = theoreticalDistance; //getMeanDistTraveled();
        scoreSoFar = meanDistance - LAMBDA*pct;

        return scoreSoFar;
    }

    public double getBestScore(GridMap gm, float[] wavefront, int numSamples, int depth)
    {
        // We (incorrectly) assert that we will never recover from being spread
        // out, much like in DART. This lets us bound our search earlier.
        getClusters();
        double maxClusterPct = (double)clusters.get(0).size()/(double)numSamples;
        return getBestScore(gm, wavefront, maxClusterPct, depth);
    }

    public double getBestScore(GridMap gm, float[] wavefront, double pct, int depth)
    {
        return getBestScore(gm, wavefront, pct, depth, 1.0);
    }

    // Get the best possible score we could still achieve with this chain of
    // behaviors. This is some combination of our distribution of XYTS along with
    // the distance we likely traveled to arrive at that distribution. We prefer
    // to travel less and arrive more reliably. Note: lower scores are BETTER
    //
    // The grid map and corresponding wavefront are used to evaluate best-case
    // distance to the goal from our current location.
    public double getBestScore(GridMap gm, float[] wavefront, double pct, int depth, double penalty)
    {
        if (behaviorScore < Double.MAX_VALUE)
            return behaviorScore;

        double meanDistance = getMeanEstimatedDistance(gm, wavefront);

        // XXX Penalty is always 0 here
        behaviorScore = meanDistance - LAMBDA*(pct*penalty); // Not perfect, but interesting
        //behaviorScore = -pct/(meanDistance+1.0);
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

    public double getMeanEstimatedDistance(GridMap gm, float[] wavefront)
    {
        if (xyts.size() < 1)
            return Double.MAX_VALUE;

        double meanDist = getMeanDistTraveled();
        double meanRemaining = getMeanDistToGoal(gm, wavefront);

        return meanDist + meanRemaining;
    }

    // Returns a number in the range of [-sqrt(2), sqrt(2)]
    public double getMeanDirectionalBonus(GridMap gm, float[] wavefront)
    {
        if (xyts.size() < 1)
            return 0;

        double mean = 0;
        for (int i = 0; i < xyts.size(); i++) {
            double[] xyt = xyts.get(i);
            int ix = (int)(Math.floor((xyt[0]-gm.x0)/gm.metersPerPixel));
            int iy = (int)(Math.floor((xyt[1]-gm.y0)/gm.metersPerPixel));
            double wfdist = (double)wavefront[iy*gm.width + ix];

            // Project forward 1 unit along theta to get bonus
            int dx = (int)(Math.round(Math.cos(xyt[2])));
            int dy = (int)(Math.round(Math.sin(xyt[2])));
            ix += dx;
            iy += dy;
            double ddist = wfdist;
            if (ix >= 0 && ix < gm.width && iy >= 0 && iy < gm.height)
                ddist = (double)wavefront[iy*gm.width + ix];
            mean += (ddist - wfdist); // Negative is better
        }

        return mean / xyts.size();
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

        assert (xyts.size() > 0);

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
        return String.format("(%f) %s until %s\n", prob, law.toString(), test.toString());
    }

    // A more useful equals call, this one is actually called when graph building.
    public boolean behaviorEquals(Behavior behavior)
    {
        if (behavior.law == null || law == null)
            return false;
        if (behavior.test == null || test == null)
            return false;
        boolean a = tagID == behavior.tagID;
        boolean b = law.equals(behavior.law);
        boolean c = test.equals(behavior.test);

        return a && b && c;
    }

    // XXX These only exist to support MonteCarloBot's hash table for class counting.
    // They do NOT generalize beyond this application, and in fact, are actively bad
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
