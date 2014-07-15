package probcog.commands.tests;

import java.util.*;

import april.util.*;

import probcog.commands.*;

public class TimeoutTest implements ConditionTest
{
    double timeout;
    Tic elapsed;

    /** Strictly for use with parameter checking */
    public TimeoutTest()
    {
    }

    public TimeoutTest(HashMap<String, TypedValue> parameters)
    {
        assert (parameters.containsKey("timeout"));
        timeout = parameters.get("timeout").getDouble();
        elapsed = new Tic();
    }

    /** Query whether or not the condition being tested for is currently true.
     *
     *  @return True if condition test is currently satisfied, else false
     **/
    public boolean conditionMet()
    {
        return elapsed.toc() > timeout;
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("timeout",
                                      TypedValue.TYPE_DOUBLE,
                                      true)); // Timeout [s]

        return params;
    }
}
