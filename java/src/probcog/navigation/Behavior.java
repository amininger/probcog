package probcog.navigation;

import java.awt.Color;
import java.util.*;

import april.util.*;
import april.vis.*;

import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;

public class Behavior
{
    Random r = new Random();

    // Control state
    public FollowWall law;                  // Law to follow
    public ClassificationCounterTest test;  // Test to check against

    // Bookkeeping for evaluating search
    double behaviorScore = Double.MAX_VALUE;
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

    public Behavior(ArrayList<double[]> xyts, ArrayList<Double> distances, FollowWall law, ClassificationCounterTest test)
    {
        assert (xyts.size() == distances.size());

        this.xyts = xyts;
        this.law = law;
        this.test = test;
        this.distances = distances;
    }

    // Get the best possible score we could still achieve with this chain of
    // behaviors. This is some combination of our distribution of XYTS along with
    // the distance we likely traveled to arrive at that distribution. Lower is
    // better, so we return distance as a negative.
    //
    // The grid map and corresponding wavefront are used to evaluate best-case
    // distance to the goal from our current location.
    public double getMaxScore(GridMap gm, float[] wavefront)
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

            meanDistance += dist + wfdist;
        }

        meanDistance /= xyts.size();

        // XXX Factor in goodness of arrival rate, too. We sort of do right now
        // by calculating a mean distance, but it's not quite what we want.
        behaviorScore = -meanDistance*getPctNearGoal(gm, wavefront);
        return behaviorScore;
    }

    public double getPctNearGoal(GridMap gm, float[] wavefront)
    {
        if (xyts.size() < 1)
            return 0;

        double GOAL_THRESH = 3.0;
        int count = 0;
        for (int i = 0; i < xyts.size(); i++) {
            double[] xyt = xyts.get(i);
            int ix = (int)(Math.floor((xyt[0]-gm.x0)/gm.metersPerPixel));
            int iy = (int)(Math.floor((xyt[1]-gm.y0)/gm.metersPerPixel));
            double wfdist = (double)wavefront[iy*gm.width + ix];
            if (wfdist < GOAL_THRESH)
                count++;
        }

        return (double)count/(double)xyts.size();
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

    public VisObject getVisObject()
    {
        VisVertexData vvd = new VisVertexData();
        for (double[] xyt: xyts)
            vvd.add(xyt, 1, 2); // Not very efficient
        return new VzPoints(vvd, new VzPoints.Style(Color.red, 5));
    }

    public String toString()
    {
        Formatter f = new Formatter();
        f.format("Follow %s until %d %s\n", law.getSide(), test.getCount(), test.getClassType());
        return f.toString();
    }
}
