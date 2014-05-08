package probcog.commands.controls;

import java.util.*;

import probcog.commands.*;

public interface ControlLaw
{
    /** Construct an instance of the ControlLaw with the supplied parameters.
     *
     *  @param parameters   A map of parameter names to values
     *
     *  @return An instance of a ControlLaw
     **/
    static public ControlLaw construct(Map<String, TypedValue> parameters);

    /** Start/stop the execution of the control law.
     *
     *  @param run  True causes the control law to begin execution, false stops it
     **/
    public void setRunning(boolean run);

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    static public Collection<TypedParameter> getParameters()
}
