package probcog.commands.controls;

import lcm.lcm.*;

import april.util.*;

import probcog.lcmtypes.*;
import probcog.commands.tests.*;

public abstract class ControlLaw implements LCMSubscriber
{
    // XXX New interface
    //
    // Constructor from control law?
    //
    // Constructor from parameters/nothing
    //
    // abstract execution/run loop...I think it should be mandated that this
    // spin off an independent thread which runs a function you are forced
    // to implement
    //
    // abstract status loop...this could be part of the run loop, possibly. Force
    // the user to further implement a status update task for SOAR.
    //
    // Ability to query parameters of control law...what settables do you have
    // and with which properties?
    // XXX This may necessitate implementing Task AND something else that defines
    // the interface to the task? e.g. ControlLawTask
    //
    // class/interface ControlLawTask implements PeriodicTask.Task {
    //      int hz; // Could this exist in an interface? getHz()?
    //      _____ getParameters();  // Returns useful names for GUI, type, ...?
    // }
    //
    LCM lcm = LCM.getSingleton();

    protected int id;
    protected String name;  // XXX Does this still serve a purpose?

    protected PeriodicTasks backgroundTasks = new PeriodicTasks(1);
    protected PeriodicTasks tasks = new PeriodicTasks(1);

    Status status = EXECUTING;
	public enum Status
    {
		EXECUTING, FINISHED, EARLY_TERM, FAILURE
	}

    class StatusTask implements PeriodicTasks.Task
    {
        public void run(double dt)
        {
            control_law_status_t cls = new control_law_status_t();

            cls.utime = TimeUtil.utime();
            cls.id = id;

            // XXX Only handles executing and finished right now. Others would
            // need to be implemented later.
            cls.status = status.name();

            // lcm.publish("SOAR_COMMAND_STATUS", cls)
            lcm.publish("CONTROL_LAW_STATUS", cls);
        }
    }

    public ControlLaw(int id, String name, PeriodicTasks.Task task, int hz)
    {
        this.id = id;
        this.name = name;

        tasks.addFixedRate(task, 1.0/hz);
        backgroundTasks.addFixedRate(new StatusTask(), 1.0/10);

        lcm.subscribe("TERMINATION_STATUS", this);

        // Start running by default
        tasks.setRunning(true);
        backgroundTasks.setRunning(true);
    }

    public ControlLaw(control_law_t controlLaw)
    {
        // XXX
    }

    public int getID()
    {
        return id;
    }

    public String getName()
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
        if ("TERMINATION_STATUS".equals(channel)) {
            control_run_status_t crs = new control_run_status_t(ins);
            // Only handle matching status messages
            if (crs.id != id) {
                return;
            }

            // If instructed to stop running, halt control tasks
            if (!crs.running) {
                status = FINISHED;
                tasks.setRunning(false);
            }
        }
    }


}
