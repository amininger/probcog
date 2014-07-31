package probcog.navigation;

import probcog.commands.*;
import probcog.commands.controls.FollowWall;
import probcog.commands.tests.ClassificationCounterTest;

public class Behavior
{
    public FollowWall law;
    public ClassificationCounterTest test;

    public Behavior(FollowWall law, ClassificationCounterTest test)
    {
        this.law = law;
        this.test = test;
    }
}
