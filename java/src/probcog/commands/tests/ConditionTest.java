package probcog.commands.tests;

import april.util.*;

import probcog.lcmtypes.*;

public abstract class ConditionTest implements LCMSubscriber
{
    // XXX New interface
    //
    // constructor for some lcm type
    //
    // constructor based on parameters of some sort (note: both of these need
    // to somehow associate the condition test with some control law
    //
    // abstract interface that launches the test monitor thread (which runs
    // some function implemented by the user).
    //
    // Much like ControlLaw, need something that exposes parameters to feed
    // into a particular task...
    LCM lcm = LCM.getSingleton();

    protected int id;
    protected String name;

    protected PeriodicTasks backgroundTasks = new PeriodicTasks(1);
    protected PeriodicTasks tasks = new PeriodicTasks(1);
    protected PeriodicTasks.Task task;  // XXX This will need to be specialized

    class StatusTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {
            // XXX May change type name, just so Soar could instantiate a
            // condition test appropriately from afar.
            condition_test_t ct = new condition_test_t();

            ct.utime = TimeUtil.utime();
            ct.id = id;
            ct.condition_met = task.conditionMet();
        }
    }

    public ConditionTest(int id, String name, PeriodicTasks.Task task, int hz)
    {
        this.id = id;
        this.name = name;   // XXX Does this need to exist, here? Mostly debugging...

        this.task = task;
        this.tasks.addFixedRate(task, 1.0/hz);

        lcm.subscribe("CONTROL_LAW_STATUS", this);

        // Set all tasks running by default
        // XXX When does a condition test die? When the task is terminated!
        this.tasks.setRunning(true);
    }

    public ConditionTest(condition_test_t test)
    {
        // XXX
    }

    public getID()
    {
        return id;
    }

    public getName()
    {
        return name;
    }

    // === LCM Handling ============================
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, LCMDataInputStream);
        } catch (IOException ex) {
            System.err.println("LCM: Error handling message on channel - "+channel);
            ex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
    {
        if ("CONTROL_LAW_STATUS".equals(channel)) {
            control_law_status_t cls = new control_law_status_t(ins);
            // Only handle matching status messages
            if (cls.id != id) {
                return;
            }

            // If associated control law stops executing, halt. Right now, we
            // just act as if this means that we finished executing cleanly, but
            // we could change this in the future.
            if (!"EXECUTING".equals(cls.status)) {
                tasks.setRunning(false);
            }
        }
    }
}
