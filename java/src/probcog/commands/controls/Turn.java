package probcog.commands.controls;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.lcmtypes.*;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.commands.*;

public class Turn implements ControlLaw
{
    static final int DD_HZ = 100;

	enum Direction { LEFT, RIGHT };
	Direction dir;

    private PeriodicTasks tasks = new PeriodicTasks(1);

    private class TurnTask implements PeriodicTasks.Task
    {
        public TurnTask()
        {
            System.out.println("Turn ready to execute");
        }

        public void run(double dt)
        {
            // Initialize diff drive with no movement
            diff_drive_t dd = new diff_drive_t();
            dd.left_enabled = dd.right_enabled = true;
            dd.left = 0;
            dd.right = 0;

            double speed = .4;

            // Change left and right wheels depending on turn direction
            if (dir.equals(Direction.RIGHT)) {
                dd.left = speed;
                dd.right = -speed;
            }
            else if (dir.equals(Direction.LEFT)) {
                dd.left = -speed;
                dd.right = speed;
            }

            publishDiff(dd);
        }
    }

    /** Strictly for use for parameter checking */
    public Turn()
    {
    }

    public Turn(Map<String, TypedValue> parameters)
    {
        System.out.println("TURN");

        // Needs a direction to turn, currently
        assert (parameters.containsKey("direction"));
        byte direction = parameters.get("direction").getByte();
        if (direction > 0) // CW
            dir = Direction.LEFT;
        else // CCW
            dir = Direction.RIGHT;
        tasks.addFixedDelay(new TurnTask(), 1.0/DD_HZ);
    }

    /** Start/stop the execution of the control law.
     *
     *  @param run  True causes the control law to begin execution, false stops it
     **/
    public void setRunning(boolean run)
    {
        tasks.setRunning(run);
    }

    /** Get the name of this control law. Mostly useful for debugging purposes.
     *
     *  @return The name of the control law
     **/
    public String getName()
    {
        return "TURN";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        ArrayList<TypedValue> options = new ArrayList<TypedValue>();
        options.add(new TypedValue((byte)-1));
        options.add(new TypedValue((byte)1));
        params.add(new TypedParameter("direction",
                                      TypedValue.TYPE_BYTE,
                                      options));
        return params;
    }

    // XXX publishDiff is in two classes - can we consolidate?
    static private void publishDiff(diff_drive_t diff_drive)
    {
        // We may get a null if there are no poses yet
        // We should throw a WRN elsewhere if that is the case
        if (diff_drive == null)
            return;

        assert(diff_drive.left <= 1 && diff_drive.left >= -1);
        assert(diff_drive.right <= 1 && diff_drive.right >= -1);

        diff_drive.utime = TimeUtil.utime();
        LCM.getSingleton().publish("DIFF_DRIVE", diff_drive);
    }
}
