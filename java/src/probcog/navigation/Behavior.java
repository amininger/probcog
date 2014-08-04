package probcog.navigation;

import java.util.*;

import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;

public class Behavior
{
    Random r = new Random();
    public ArrayList<double[]> xyts = new ArrayList<double[]>();
    public FollowWall law;                  // Law to follow
    public ClassificationCounterTest test;  // Test to check against

    public Behavior(ArrayList<double[]> xyts, FollowWall law, ClassificationCounterTest test)
    {
        this.xyts = xyts;
        this.law = law;
        this.test = test;
    }

    public double[] getXYT()
    {
        assert (xyts.size() > 0);
        return xyts.get(0);
    }

    public double[] randomXYT()
    {
        assert (xyts.size() > 0);
        return xyts.get(r.nextInt(xyts.size()));
    }

    public String toString()
    {
        Formatter f = new Formatter();
        f.format("Follow %s until %s\n", law.getSide(), test.getClassType());
        return f.toString();
    }
}
