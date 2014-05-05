package probcog.commands.controls;

import probcog.lcmtypes.control_law_t;

public class ControlLawFactory{
	public static ControlLaw construct(control_law_t controlLaw){
		if(controlLaw.name.equals("turn")){
			return new Turn(controlLaw);
		} else if(controlLaw.name.equals("drive-forward")){
            System.out.println("Control Law Factory saw drive-forward");
			return new DriveForward(controlLaw);
		}

		return null;
	}
}
