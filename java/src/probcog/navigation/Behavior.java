package probcog.navigation;

import java.awt.Color;
import java.util.*;

import april.vis.*;

import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;

public class Behavior
{
    Random r = new Random();
    public ArrayList<double[]> xyts = new ArrayList<double[]>();
    public FollowWall law;                  // Law to follow
    public ClassificationCounterTest test;  // Test to check against
    public double distanceTraveled;

    public Behavior(ArrayList<double[]> xyts, double dist, FollowWall law, ClassificationCounterTest test)
    {
        this.xyts = xyts;
        this.law = law;
        this.test = test;
        this.distanceTraveled = dist;
    }

    public double[] getXYT()
    {
        assert (xyts.size() > 0);
        return xyts.get(0);
    }

    // Uniformly sample an XYT from the samples
    public double[] randomXYT()
    {
        assert (xyts.size() > 0);
        return xyts.get(r.nextInt(xyts.size()));
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
