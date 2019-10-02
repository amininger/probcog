package soargroup.mobilesim.commands.tests;

import java.io.*;
import java.util.*;

import lcm.lcm.*;
import april.jmat.*;
import april.util.*;
import probcog.classify.TagHistory;
import soargroup.mobilesim.commands.*;
import soargroup.mobilesim.lcmtypes.*;
import magic2.lcmtypes.*;

public class ClassificationCounterTest implements ConditionTest, LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    // Default parameters...should these be settable?
    static final double CONFIDENCE_THRESHOLD = 0.8;
    static final double ORIENTATION_THRESHOLD = Math.toRadians(15);

    private int goalTag = -1;
    boolean stoppedAtRightTag = false;

    private int goalCount = 0;
    // By default, set to -Pi. Any value <= -3.14 will be treated as a default
    // value in which object position relative to the robot is not relevant. In
    // this mode, objects will merely be counted.
    private double orientation = -Math.PI;
    private String classType = "";
    private HashMap<Integer, DetectionRecord> observed;
    private HashSet<Integer> ignore = new HashSet<Integer>();

    class DetectionRecord
    {
        public int n;
        public double mean;
        boolean metOrientation;

        public DetectionRecord()
        {
            n = 0;
            mean = 0;
            if (orientation <= -3.14)
                metOrientation = true;
            else
                metOrientation = false;
        }

        public void addSample(tag_classification_t classy)
        {
            n++;
            mean = mean + 1.0*(classy.confidence-mean)/n;

            if (metOrientation)
                return;

            // Determine is the object was positioned correctly
            // to meet our relative orientation requirements
            if (Math.abs(MathUtil.mod2pi(classy.xyzrpy[5] - orientation)) < ORIENTATION_THRESHOLD) {
                System.out.println("ONE");
                metOrientation = true;
            }
        }

        public void addSample(String label)
        {
            n++;
            mean = 1.0; // Very confident for now.

            if (metOrientation)
                return;
        }
    }

    /** Strictly for use for parameter checking */
    public ClassificationCounterTest()
    {
    }

    public ClassificationCounterTest(Map<String, TypedValue> parameters)
    {
        //System.out.println("CLASSIFICATION COUNTER TEST");

        assert (parameters.containsKey("count"));
        goalCount = parameters.get("count").getInt();

        assert (parameters.containsKey("class"));
        classType = parameters.get("class").toString();

        // Orientation relative to us...defaults to irrelevant
        if (parameters.containsKey("orientation"))
            orientation = parameters.get("orientation").getDouble();

        // Optional specification of a goal tag...useful for sim eval
        if (parameters.containsKey("goal-tag"))
            goalTag = parameters.get("goal-tag").getInt();

        // Hidden parameters. Used to hack in tag ignorance
        for (String key: parameters.keySet()) {
            if (!key.startsWith("ignore"))
                continue;
            ignore.add(parameters.get(key).getInt());
        }

        observed = new HashMap<Integer, DetectionRecord>();

        // Initialize
        // XXX - Here we need to spin up a classifier and tell it what to look for
    }

    //public ClassificationCounterTest clone()
    public ConditionTest copyCondition()
    {
        ClassificationCounterTest test = new ClassificationCounterTest();
        test.goalCount = goalCount;
        test.classType = classType;
        test.orientation = orientation;
        test.observed = new HashMap<Integer, DetectionRecord>();
        test.goalTag = goalTag;
        for (Integer i: ignore)
            test.ignore.add(i);

        return test;
    }

    public void setRunning(boolean run)
    {
        if (run) {
            lcm.subscribe("CLASSIFICATIONS", this);
        } else {
            lcm.unsubscribe("CLASSIFICATIONS", this);
        }
    }

    /** Query whether or not the condition being tested for is currently true.
     *
     *  @return True if condition test is currently satisfied, else false
     **/
    synchronized public boolean conditionMet()
    {
        int count = getCurrentCount();
        return count >= goalCount;
    }

    public boolean conditionMetCorrectly()
    {
        return stoppedAtRightTag;
    }

    public String getClassType()
    {
        return classType;
    }

    public int getCount()
    {
        return goalCount;
    }

    public String toString()
    {
        String temp = String.format("%d %s", getCount(), getClassType());
        if (ignore.size() > 0) {
            temp += " ignoring ";
            for (Integer i: ignore) {
                temp += i + ", ";
            }
        }
        return temp;
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("count",
                                      TypedValue.TYPE_INT,
                                      true));
        params.add(new TypedParameter("class",
                                      TypedValue.TYPE_STRING,
                                      true));
        params.add(new TypedParameter("orientation",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue(-Math.PI),
                                      new TypedValue(Math.PI),
                                      false));
        params.add(new TypedParameter("goal-tag",
                                      TypedValue.TYPE_INT,
                                      false));
        return params;
    }

    public condition_test_t getLCM()
    {
        condition_test_t ct = new condition_test_t();
        ct.name = "count";
        ct.num_params = 4+ignore.size();
        ct.param_names = new String[ct.num_params];
        ct.param_values = new typed_value_t[ct.num_params];
        ct.param_names[0] = "count";
        ct.param_values[0] = new TypedValue(goalCount).toLCM();
        ct.param_names[1] = "class";
        ct.param_values[1] = new TypedValue(classType).toLCM();
        ct.param_names[2] = "orientation";
        ct.param_values[2] = new TypedValue(orientation).toLCM();
        ct.param_names[3] = "goal-tag";
        ct.param_values[3] = new TypedValue(goalTag).toLCM();
        int idx = 0;
        for (Integer i: ignore) {
            ct.param_names[4+idx] = "ignore_"+i;
            ct.param_values[4+idx] = new TypedValue(i).toLCM();
            idx++;
        }

        // Not used
        ct.compare_type = condition_test_t.CMP_GT;
        ct.compared_value = new TypedValue(0).toLCM();

        return ct;
    }

    public int hashCode()
    {
        return classType.hashCode() ^ (new Integer(goalCount)).hashCode();
    }

    public boolean equals(Object o)
    {
        if (o == null)
            return false;
        if (!(o instanceof ClassificationCounterTest))
            return false;
        ClassificationCounterTest cct = (ClassificationCounterTest)o;
        return classType.equals(cct.classType) && (goalCount == cct.goalCount);
    }

    // === Sample adding/tracking ============================================
    synchronized public void addSample(tag_classification_t classy)
    {
        // XXX On real robot, will we need independent tag tracking here, or
        // in multiple places? Probably want a unified front...for now, hacking
        // stuff in.

        if (ignore.contains(classy.tag_id))
            return;

        synchronized (observed) {
            if (classType.equals(classy.name)) {
                if (!observed.containsKey(classy.tag_id)) {
                    observed.put(classy.tag_id, new DetectionRecord());
                }
                observed.get(classy.tag_id).addSample(classy);
            }
        }
    }

    synchronized public void addSample(int id, String label)
    {
        if (ignore.contains(id))
            return;

        synchronized (observed) {
            if (classType.equals(label)) {
                if (!observed.containsKey(id)) {
                    observed.put(id, new DetectionRecord());
                }
                observed.get(id).addSample(label);
            }
        }

        //System.out.printf("%d: %d, %s\n", goalTag, id, label);
        int currCount = getCurrentCount();
        if (currCount >= goalCount && id == goalTag) {
            stoppedAtRightTag = true;   // Only affects sim, for the most part
        }
        //System.out.println(stoppedAtRightTag);
    }

    public int getCurrentCount()
    {
        int count = 0;
        synchronized (observed) {
            for (DetectionRecord d: observed.values()) {
                // Compute a sample weighted confidence. This helps account for
                // some of the noise in sampling. Chosen fairly arbitrarily to
                // heavily penalize few sample while minimally penalizing many samples,
                // but could be revisited
                //double conf = (1.0 - 1.0/d.n)*d.mean;
                double conf = d.mean;   // XXX
                if (conf > CONFIDENCE_THRESHOLD && d.metOrientation)
                    count++;

            }
        }
        return count;
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.err.println("WRN: Error reading channel "+channel+": "+ex);
        }
    }

    void messageReceivedEx(LCM lcm, String channel,
            LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("CLASSIFICATIONS")) {
            tag_classification_list_t msg = new tag_classification_list_t(ins);

            for(tag_classification_t classy : msg.classifications) {
                // Keep running average of "quality" of detection, with a penalty
                // for not having multiple observations coming into play during
                // evaluation in conditionMet()
                addSample(classy);
            }
        }
    }
}
