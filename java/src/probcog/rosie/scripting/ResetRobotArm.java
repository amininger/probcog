package probcog.rosie.scripting;

import lcm.lcm.LCM;
import probcog.lcmtypes.robot_command_t;
import april.util.TimeUtil;

public class ResetRobotArm implements UiCommand {
	@Override
	public void execute() {
		robot_command_t command = new robot_command_t();
		command.utime = TimeUtil.utime();
		command.action = "RESET";
		command.dest = new double[6];
		LCM.getSingleton().publish("ROBOT_COMMAND", command);
	}
}
