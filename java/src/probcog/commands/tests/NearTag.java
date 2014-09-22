package probcog.commands.tests;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;

import probcog.commands.*;
import probcog.lcmtypes.*;

public class NearTag implements ConditionTest, LCMSubscriber
{
    static final double DEFAULT_STOPPING_DISTANCE = 0.25;

    LCM lcm = LCM.getSingleton();

    double closestDistance = Double.MAX_VALUE;
    double stoppingDistance = DEFAULT_STOPPING_DISTANCE;
    int tagID = -1;

    public NearTag()
    {
    }

    public NearTag(HashMap<String, TypedValue> parameters)
    {
        assert (parameters.containsKey("id"));
        tagID = parameters.get("id").getInt();

        if (parameters.containsKey("distance"))
            stoppingDistance = parameters.get("distance").getDouble();

        lcm.subscribe("CLASSIFICATIONS", this);
    }

    public void processTag(classification_t c)
    {
        if (c == null || c.id != tagID)
            return;

        closestDistance = LinAlg.magnitude(LinAlg.resize(c.xyzrpy, 2));
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.startsWith("CLASSIFICATIONS")) {
                classification_list_t cl = new classification_list_t(ins);

                classification_t c = null;
                for (int i = 0; i < cl.num_classifications; i++) {
                    if (tagID == cl.classifications[i].id) {
                        c = cl.classifications[i];
                        break;
                    }
                }

                processTag(c);
            }
        } catch (IOException ex) {
            System.err.println("ERR: Could not handle message on channel - "+channel);
            ex.printStackTrace();
        }
    }

    /** Query whether or not the condition being tested for is currently true.
     *
     *  @return True if condition test is currently satisfied, else false
     **/
    public boolean conditionMet()
    {
        return closestDistance < stoppingDistance;
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("id",
                                      TypedValue.TYPE_INT,
                                      true));
        params.add(new TypedParameter("distance",
                                      TypedValue.TYPE_DOUBLE,
                                      false));

        return params;
    }

    public String toString()
    {
        return String.format("Tag %d", tagID);
    }

    public condition_test_t getLCM()
    {
        condition_test_t ct = new condition_test_t();
        ct.name = "near-tag";
        ct.num_params = 2;
        ct.param_names = new String[ct.num_params];
        ct.param_values = new typed_value_t[ct.num_params];
        ct.param_names[0] = "id";
        ct.param_values[0] = new TypedValue(tagID).toLCM();
        ct.param_names[1] = "distance";
        ct.param_values[1] = new TypedValue(stoppingDistance).toLCM();

        // Not used
        ct.compare_type = condition_test_t.CMP_GT;
        ct.compared_value = new TypedValue(0).toLCM();

        return ct;
    }
}
