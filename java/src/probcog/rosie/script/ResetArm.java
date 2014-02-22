package probcog.rosie.script;

import lcm.lcm.LCM;
import probcog.lcmtypes.robot_command_t;
import april.util.TimeUtil;
import probcog.rosie.Rosie;

public class ResetArm implements UiCommand {
	@Override
	public void execute() {
		robot_command_t command = new robot_command_t();
		command.utime = Rosie.GetSoarTime();
		command.action = "RESET";
		command.dest = new double[6];
		LCM.getSingleton().publish("ROBOT_COMMAND", command);
	}
}
