package probcog.commands;

import java.util.*;

import lcm.lcm.*;

import april.util.*;

import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.lcmtypes.*;

/** The command coordinator is responsible for the creation of new control
 *  law requests, condition tests, registration of tests as termination
 *  conditions for control law(s), an in general, dealing with requests to the
 *  robotic system to perform some action.
 *
 *  For example, a request to "Turn right after the green painting" could
 *  result in the following events:
 *  -Spawn a control law that keeps the robot driving down a corridor
 *  -Turn on a classifier for identifying green paintings
 *  -Spawn a condition test that indicates when we are "after" a green painting
 *   (which may involve spatial reasoning on Soar or some other classifier,
 *    even!)
 *  -Spawn a turn detecting condition test (specifically for right, or possibly
 *   with more back and forth with Soar, etc.)
 *
 *  Condition Tests could feasibly be compositions of other Condition Tests. For
 *  example, maybe we only care if one of two tests is true. Maybe we need
 *  several conditions to be true simultaneously! In this case, the highest
 *  level condition test is all that's registered with the Coordinator, while
 *  it internally manages its own condition tests.
 **/
public class CommandCoordinator
{
    LCM lcm = LCM.getSingleton();

    private static int controlLawIDCounter = 0;
    private static int conditionTestIDCounter = 0;

    public enum Status
    {
        EXECUTING, FAILURE, SUCCESS, UNKNOWN
    }

    private class ControlLawRecord
    {
        public ControlLaw controlLaw;
        public Status executionStatus;

        public ControlLawRecord(ControlLaw controlLaw)
        {
            this.controlLaw = controlLaw;
            this.executionStatus = Status.EXECUTING;
        }
    }
    Map<Integer, ControlLawRecord> controlLaws = Collections.synchronizedMap(new HashMap<Integer, ControlLawRecord>());

    // XXX Need records here, too?
    Map<Integer, ConditionTest> conditionTests = Collections.synchronizedMap(new HashMap<Integer, ConditionTest>());

    private class TerminationCondition
    {
        public Integer id; // ID of control law
        public Status terminationStatus;

        public TerminationCondition(int id, Status status)
        {
            this.id = id;
            this.terminationStatus = status;
        }
    }
    // Condition test ID -> relevant status/ID information for one or more control laws
    Map<Integer, ArrayList<TerminationCondition> > terminationConditions = Collections.synchronizedMap(new HashMap<Integer, ArrayList<TerminationCondition> >());

    PeriodicTasks tasks = new PeriodicTasks(1);
    private class UpdateTask implements PeriodicTasks.Task
    {
        /** Query control laws for statuses, manage termination conditions, etc. */
        public void run(double dt)
        {
            // XXX TODO: When do we clean up after these things?

            // Check termination conditions. If any are met, handle control law
            // updates accordingly. For example, some conditions might be
            // indicative of failure.
            Set<Integer> keys = conditionTests.keySet();
            synchronized (conditionTests) {
                for (Integer key: keys) {
                    ConditionTest test = conditionTests.get(key);
                    synchronized (terminationConditions) {
                        if (test.conditionMet() && terminationConditions.containsKey(key)) {
                            ArrayList<TerminationCondition> terms = terminationConditions.get(key);
                            for (TerminationCondition term: terms) {
                                // XXX Multiple condition interactions?
                                if (controlLaws.containsKey(term.id)) {
                                    controlLaws.get(term.id).controlLaw.setRunning(false);
                                    controlLaws.get(term.id).executionStatus = term.terminationStatus;
                                } else {
                                    // XXX
                                }
                            }
                        }
                        // Broadcast relevant status information to those who care
                        // XXX Who cares?
                    }
                }
            }

            // Send out status messages via LCM.
            keys = controlLaws.keySet();
            synchronized (controlLaws) {
                control_law_status_list_t sl = new control_law_status_list_t();
                sl.utime = TimeUtil.utime();
                sl.nstatuses = keys.size();
                sl.statuses = new control_law_status_t[sl.nstatuses];
                int idx = 0;
                for (Integer key: keys) {
                    ControlLawRecord record = controlLaws.get(key);

                    control_law_status_t s = new control_law_status_t();
                    s.id = key;
                    s.name = record.controlLaw.getName();   // XXX
                    s.status = record.executionStatus.name();

                    sl.statuses[idx++] = s;
                }
                lcm.publish("CONTROL_LAW_STATUS_TX", sl);
            }
        }
    }

    public CommandCoordinator()
    {
        int hz = 100;
        tasks.addFixedRate(new UpdateTask(), 1.0/hz);
        tasks.setRunning(true);
    }

    /** Register a control law with the Coordinator.
     *
     *  @param controlLaw       Control law to be registered
     *
     *  @return ID assigned to law
     **/
    public int registerControlLaw(ControlLaw controlLaw)
    {
        synchronized (controlLaws) {
            int id = controlLawIDCounter++;
            controlLaws.put(id, new ControlLawRecord(controlLaw));
            controlLaw.setRunning(true);
            System.out.printf("Registered and started Law <%d>\n", id);

            // Automatically insert a failure condition for getting too close
            // to an obstacle...
            //int termId = registerConditionTest(new ObstacleTest(new HashMap<String, TypedValue>()));
            //registerTerminationCondition(termId, id, CommandCoordinator.Status.FAILURE);
            HashMap<String, TypedValue> params = new HashMap<String, TypedValue>();
            params.put("timeout", new TypedValue(300.0));   // 5 minutes XXX
            int termId = registerConditionTest(new TimeoutTest(params));
            registerTerminationCondition(termId, id, CommandCoordinator.Status.FAILURE);

            return id;
        }
    }

    /** Destroy the control law associated with the given ID.
     *
     *  @param id   ID of control law to destroy
     *
     *  @return     True is matching control law was destroyed, else false
     **/
    public boolean destroyControlLaw(Integer id)
    {
        synchronized (controlLaws) {
            ControlLawRecord record = controlLaws.remove(id);
            if (record != null) {
                record.controlLaw.setRunning(false);
                System.out.printf("Destroyed Law <%d>\n", id);

                // Go through and clean up termination condition stuff.
                // XXX Assumes that condition tests never apply to more than one law
                // at the same time.
                ArrayList<Integer> condIDs = new ArrayList<Integer>();
                synchronized (terminationConditions) {
                    for (Integer key: terminationConditions.keySet()) {
                        ArrayList<TerminationCondition> terms = terminationConditions.get(key);
                        for (int i = 0; i < terms.size(); i++) {
                            if (terms.get(i).id == id) {
                                condIDs.add(key);
                            }
                        }
                    }
                }

                for (Integer key: condIDs)
                    destroyConditionTest(key);
            }
            return record != null;
        }
    }

    /** Register a condition test with the Coordinator.
     *
     *  @param conditionTest    Condition test to be registered
     *
     *  @return ID assigned to test
     **/
    public int registerConditionTest(ConditionTest conditionTest)
    {
        synchronized (conditionTests) {
            int id = conditionTestIDCounter++;
            conditionTests.put(id, conditionTest);
            conditionTest.setRunning(true);
            System.out.printf("Registered Test <%d>\n", id);
            return id;
        }
    }

    /** Destroy the condition test associated with the given ID.
     *
     *  @param id   ID of condition test to destroy
     *
     *  @return     True is matching condition test was destroyed, else false
     **/
    public boolean destroyConditionTest(int id)
    {
        synchronized (conditionTests) {
            synchronized (terminationConditions) {
                ConditionTest conditionTest = conditionTests.remove(id);
                conditionTest.setRunning(false);
                terminationConditions.remove(id);
                if (conditionTest != null) {
                    System.out.printf("Destroyed Test <%d>\n", id);
                }
                return conditionTest != null;
            }
        }
    }

    /** Register a condition test as a termination condition for a control law.
     *  Also registers the appropriate termination status for the control law.
     *  For example, tests will typically indicate successful execution of a
     *  particular control law. However, some tests might only be expected to
     *  trigger in the case of a failure (e.g. "You've gone too far").
     *
     *  @param testID           ID of condition test to use as terminator
     *  @param lawID            ID of law to terminate when test is true
     *  @param status           Corresponding status of a control law when condition is met
     *
     **/
    public void registerTerminationCondition(int testID, int lawID, Status status)
    {
        synchronized (terminationConditions) {
            if (!terminationConditions.containsKey(testID)) {
                terminationConditions.put(testID, new ArrayList<TerminationCondition>());
            }
            ArrayList<TerminationCondition> conds = terminationConditions.get(testID);
            conds.add(new TerminationCondition(lawID, status));
        }
            System.out.printf("Registered Test <%d> to Control Law <%d> with termination status <%s>\n",
                              testID,
                              lawID,
                              status.name());
    }
}
