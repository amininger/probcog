package soargroup.mobilesim.commands.controls;

import java.util.*;

import soargroup.mobilesim.commands.*;
import soargroup.mobilesim.robot.control.*;
import magic2.lcmtypes.*;

// XXX Temporary port to new control law implementation. This is just a water-
// through-the-pipes implementation.
public class PutDown implements ControlLaw
{
    Params storedParams = Params.makeParams();

    /** Strictly for creating instances for parameter checks */
    public PutDown()
    {
    }

    public PutDown(Map<String, TypedValue> parameters)
    {
        System.out.println("PUT DOWN");
    }

    /** Start/stop the execution of the control law.
     *
     *  @param run  True causes the control law to begin execution, false stops it
     **/
    public void setRunning(boolean run)
    {
       
    }

    /** Get the name of this control law. Mostly useful for debugging purposes.
     *
     *  @return The name of the control law
     **/
    public String getName()
    {
        return "PUT_DOWN";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        // No parameters, so this can just return an empty container
    	ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
    	return params;
    }

	@Override
	public diff_drive_t drive(DriveParams params) {
		return null;
	}
}
