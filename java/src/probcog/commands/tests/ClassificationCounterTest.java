package probcog.commands.tests;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.commands.*;
import probcog.lcmtypes.*;

public class ClassificationCounterTest implements ConditionTest, LCMSubscriber
{
    private double goalCount = 0;
    private double currentCount = 0;
    private String classType = "";
    private HashSet<Integer> observedIDs;

    /** Strictly for use for parameter checking */
    public ClassificationCounterTest()
    {
    }

    public ClassificationCounterTest(Map<String, TypedValue> parameters)
    {
        System.out.println("CLASSIFICATION COUNTER TEST");

        assert (parameters.containsKey("count"));
        goalCount = parameters.get("count").getDouble();

        assert (parameters.containsKey("class"));
        classType = parameters.get("class").toString();

        System.out.println("Found class");
        observedIDs = new HashSet<Integer>();

        // Initialize
        // XXX - Here we need to spin up a classifier and tell it what to look for

        LCM.getSingleton().subscribe("CLASSIFICATIONS", this);
    }

    /** Query whether or not the condition being tested for is currently true.
     *
     *  @return True if condition test is currently satisfied, else false
     **/
    public boolean conditionMet()
    {
        return currentCount >= goalCount;
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("count",
                                      TypedValue.TYPE_DOUBLE));

        params.add(new TypedParameter("class",
                                      TypedValue.TYPE_STRING));
        return params;
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.err.println("WRN: Error reading channel "+channel+": "+ex);
        }
    }

    synchronized void messageReceivedEx(LCM lcm, String channel,
            LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("CLASSIFICATIONS")) {
            classifications_t msg = new classifications_t(ins);

            // If correct category and previously unobserved, increment counter
            if(msg.name.equals(classType) && !observedIDs.contains(msg.id)) {
                observedIDs.add(msg.id);
                currentCount ++;
            }
        }
    }
}
