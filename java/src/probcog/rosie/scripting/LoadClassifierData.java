package probcog.rosie.scripting;

import lcm.lcm.LCM;
import probcog.lcmtypes.perception_command_t;
import probcog.lcmtypes.robot_command_t;
import april.util.TimeUtil;

public class LoadClassifierData implements UiCommand {
	@Override
	public void execute() {
		perception_command_t cmd = new perception_command_t();
		cmd.utime = TimeUtil.utime();
		cmd.command = "LOAD_CLASSIFIERS=default";
        LCM.getSingleton().publish("PERCEPTION_COMMAND", cmd);
	}
}
