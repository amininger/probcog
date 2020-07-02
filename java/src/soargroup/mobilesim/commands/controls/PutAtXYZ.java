package soargroup.mobilesim.commands.controls;

import java.util.*;

import soargroup.mobilesim.commands.*;
import soargroup.mobilesim.lcmtypes.diff_drive_t;

public class PutAtXYZ implements ControlLaw
{
    Params storedParams = Params.makeParams();

    /** Strictly for creating instances for parameter checks */
    public PutAtXYZ()
    {
    }

    public PutAtXYZ(Map<String, TypedValue> parameters)
    {
        System.out.println("PUT DOWN AT {" + parameters.get("x").toString() + ", " + 
				parameters.get("y").toString() + ", " + parameters.get("z").toString() + "}");
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
        return "PUT_AT_XYZ";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        // No parameters, so this can just return an empty container
    	ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("x", TypedValue.TYPE_DOUBLE, true));
        params.add(new TypedParameter("y", TypedValue.TYPE_DOUBLE, true));
        params.add(new TypedParameter("z", TypedValue.TYPE_DOUBLE, true));
		// If given, will move the given object instead of the grabbed one
    	params.add(new TypedParameter("object-id", TypedValue.TYPE_INT, false));

    	return params;
    }

	@Override
	public diff_drive_t drive(DriveParams params) {
		return null;
	}
}
