package soargroup.mobilesim.commands.controls;

import java.util.*;

import soargroup.mobilesim.commands.*;
import soargroup.mobilesim.lcmtypes.diff_drive_t;

/** Tells the robot to put the held object onto another one **/
public class PutOnObject extends ControlLaw {
    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable, immutable collection of all possible parameters
     **/
	private static List<TypedParameter> parameters = null;
    public static Collection<TypedParameter> getParameters()
    {
		if(parameters == null){
			ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
			params.add(new TypedParameter("destination-id", TypedValue.TYPE_INT, true));
			params.add(new TypedParameter("relation", TypedValue.TYPE_STRING, false));
			parameters = Collections.unmodifiableList(params);
		}
		return parameters;
    }

	// The coordinate to put the object at
	public final Integer destinationId;
	public final String relation;

    public PutOnObject(Map<String, TypedValue> parameters) {
		super(parameters);
		ControlLaw.validateParameters(parameters, PutOnObject.getParameters());

		destinationId = parameters.get("destination-id").getInt();
		if(parameters.containsKey("relation")){
			relation = parameters.get("relation").toString();
		} else {
			relation = "on";
		}
    }

	@Override
    public String getName() { return "PutOnObject"; }

	@Override
	public String toString() { 
		return String.format("PutOnObject( %s(%d) )", relation, destinationId); 
	}
}
