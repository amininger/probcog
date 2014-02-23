package probcog.commands.controls;

import probcog.lcmtypes.control_law_t;
import probcog.lcmtypes.typed_value_t;

public class DriveForward extends ControlLaw{
	public DriveForward(control_law_t controlLaw){
		super(controlLaw);
	}

	@Override
	public void execute(){
		// TODO: Code to make sure the robot is driving forward
	}

	@Override
	public ControlLaw.Status getStatus(){
		// TODO: may return EARLY_TERM or FAILURE if 
		//   something went wrong

		if(termCond.evaluate() == true){
			return ControlLaw.Status.FINISHED;
		}
		return ControlLaw.Status.EXECUTING;
	}
}
