package probcog.commands.tests;

import java.util.*;

import probcog.commands.*;

public interface ConditionTest
{
    /** Construct an instance of the condition test with the supplied parameters.
     *
     *  @param parameters   A map of parameter names to values
     *
     *  @return An instance of a ConditionTest
     **/
    static public ConditionTest construct(Map<String, TypedValue> parameters);

    /** Start/stop the execution of the condition test.
     *
     *  @param run  True causes the condition test to begin execution, false stops it
     **/
    public void setRunning(boolean run);

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    static public Collection<TypedParameter> getParameters();
}
