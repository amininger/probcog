package probcog.rosie.scripting;

import lcm.lcm.LCM;
import april.util.TimeUtil;
import probcog.lcmtypes.perception_command_t;
import probcog.rosie.perception.WorldModel;

public class PointAtObject implements UiCommand {
	private int objectId;
	
	public PointAtObject(int id) {
		objectId = id;
	}
	
	@Override
	public void execute() {
		perception_command_t command = new perception_command_t();
		command.utime = TimeUtil.utime();
		command.command = "select=" + objectId;
		LCM.getSingleton().publish("GUI_COMMAND", command);
		// XXX: Fix
		//WorldModel.Singleton().setPointedObjectID(objectId);
	}
}
