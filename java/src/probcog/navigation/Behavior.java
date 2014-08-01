package probcog.navigation;

import java.util.Formatter;

import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;

public class Behavior
{
    public double[] xyt;                    // Estimated final location
    public FollowWall law;                  // Law to follow
    public ClassificationCounterTest test;  // Test to check against

    public Behavior(double[] xyt, FollowWall law, ClassificationCounterTest test)
    {
        this.xyt = xyt;
        this.law = law;
        this.test = test;
    }

    public String toString()
    {
        Formatter f = new Formatter();
        f.format("Follow %s until %s\n", law.getSide(), test.getClassType());
        return f.toString();
    }
}
