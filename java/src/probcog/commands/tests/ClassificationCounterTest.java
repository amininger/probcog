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
    // Default parameters...should these be settable?
    static final double CONFIDENCE_THRESHOLD = 0.8;

    private int goalCount = 0;
    private String classType = "";
    private HashMap<Integer, DetectionRecord> observed;

    class DetectionRecord
    {
        public int n;
        public double mean;

        public DetectionRecord()
        {
            n = 0;
            mean = 0;
        }

        public void addSample(double v)
        {
            n++;
            mean = mean + 1.0*(v-mean)/n;
        }
    }

    /** Strictly for use for parameter checking */
    public ClassificationCounterTest()
    {
    }

    public ClassificationCounterTest(Map<String, TypedValue> parameters)
    {
        System.out.println("CLASSIFICATION COUNTER TEST");

        assert (parameters.containsKey("count"));
        goalCount = parameters.get("count").getInt();

        assert (parameters.containsKey("class"));
        classType = parameters.get("class").toString();

        observed = new HashMap<Integer, DetectionRecord>();

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
        int count = 0;
        for (DetectionRecord d: observed.values()) {
            // Compute a sample weighted confidence. This helps account for
            // some of the noise in sampling. Chosen fairly arbitrarily to
            // heavily penalize few sample while minimally penalizing many samples,
            // but could be revisited
            double conf = (1.0 - 1.0/d.n)*d.mean;
            if (conf > CONFIDENCE_THRESHOLD)
                count++;
        }

        return count >= goalCount;
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("count",
                                      TypedValue.TYPE_INT));

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

            // Keep running average of "quality" of detection, with a penalty
            // for not having multiple observations coming into play during
            // evaluation in conditionMet()
            if (classType.equals(msg.name)) {
                if (!observed.containsKey(msg.id)) {
                    observed.put(msg.id, new DetectionRecord());
                }
                observed.get(msg.id).addSample(msg.confidence);
            }
        }
    }
}
