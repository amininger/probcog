package probcog.commands.controls;

import probcog.lcmtypes.control_law_t;
import probcog.lcmtypes.typed_value_t;

import probcog.commands.TypedValue;

public class Turn extends ControlLaw{
	enum Direction { LEFT, RIGHT };

	Direction dir;

	public Turn(control_law_t controlLaw){
		super(controlLaw);

		dir = Direction.LEFT;

		// Parameters:
		//   direction = { left, right }
		for(int p = 0; p < controlLaw.num_params; p++){
			if(controlLaw.param_names[p].equals("direction")){
				String value = TypedValue.unwrapString(controlLaw.param_values[p]);
				if(value.equals("left")){
					dir = Direction.LEFT;
				} else if(value.equals("right")){
					dir = Direction.RIGHT;
				}
			}
		}
	}

	@Override
	public void execute(){
		// TODO: Code to make the robot turn
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
