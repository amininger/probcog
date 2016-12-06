package probcog.rosie.scripting;

import java.io.IOException;
import java.util.Random;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import april.util.TimeUtil;
import probcog.lcmtypes.*;
import probcog.rosie.perception.WorldModel;


public class ResetEnvironmentState implements UiCommand {
	private Boolean stoveDoorState; // 0 is closed; 1 is open
	private Boolean stoveHeatState; // 0 is off; 1 is on
	private Boolean armState; // O is not-held; 1 is held
	private Boolean pantryDoorState; // 0 is closed; 1 is open
	private Integer objectId;
	private Random random;
	private Integer held;
	private boolean objectCookedState;
	
	public ResetEnvironmentState(Integer id, Integer heldObject){
		objectId = id;
		held = heldObject;
		random = new Random();
	}
	
	@Override
	public void execute() {
		
		new ResetRobotArm().execute();
		stoveHeatState = random.nextBoolean();
		pantryDoorState = random.nextBoolean();
		objectCookedState = random.nextBoolean();
		armState = random.nextBoolean();
		
		set_state_command_t commandObjectState1 = new set_state_command_t();
		commandObjectState1.utime = TimeUtil.utime();
		commandObjectState1.state_name = "meat1";
		commandObjectState1.state_val = "raw1";
		commandObjectState1.obj_id = 5;
		LCM.getSingleton().publish("SET_STATE_COMMAND", commandObjectState1);
		
		set_state_command_t commandObjectState2 = new set_state_command_t();
		commandObjectState2.utime = TimeUtil.utime();
		commandObjectState2.state_name = "meat1";
		commandObjectState2.state_val = "raw1";
		commandObjectState2.obj_id = 6;
		LCM.getSingleton().publish("SET_STATE_COMMAND", commandObjectState2);
		
		set_state_command_t commandObjectState3 = new set_state_command_t();
		commandObjectState3.utime = TimeUtil.utime();
		commandObjectState3.state_name = "meat1";
		commandObjectState3.state_val = "raw1";
		commandObjectState3.obj_id = 7;
		LCM.getSingleton().publish("SET_STATE_COMMAND", commandObjectState3);
		
		set_state_command_t commandObjectState4 = new set_state_command_t();
		commandObjectState4.utime = TimeUtil.utime();
		commandObjectState4.state_name = "meat1";
		commandObjectState4.state_val = "raw1";
		commandObjectState4.obj_id = 8;
		LCM.getSingleton().publish("SET_STATE_COMMAND", commandObjectState4);
		
		set_state_command_t commandStoveHeat = new set_state_command_t();
		if (stoveHeatState){
			commandStoveHeat.utime = TimeUtil.utime();
			commandStoveHeat.state_name = "activation1";
			commandStoveHeat.state_val = "on2";
			commandStoveHeat.obj_id = 4;
			LCM.getSingleton().publish("SET_STATE_COMMAND", commandStoveHeat);
		}
		else {
			commandStoveHeat.utime = TimeUtil.utime();
			commandStoveHeat.state_name = "activation1";
			commandStoveHeat.state_val = "off2";
			commandStoveHeat.obj_id = 4;
			LCM.getSingleton().publish("SET_STATE_COMMAND", commandStoveHeat);
		}
		
		set_state_command_t commandPantryDoor = new set_state_command_t();
		if (pantryDoorState){
			commandPantryDoor.utime = TimeUtil.utime();
			commandPantryDoor.state_name = "door1";
			commandPantryDoor.state_val = "open2";
			commandPantryDoor.obj_id = 1;
			LCM.getSingleton().publish("SET_STATE_COMMAND", commandPantryDoor);
		}
		else {
			commandPantryDoor.utime = TimeUtil.utime();
			commandPantryDoor.state_name = "door1";
			commandPantryDoor.state_val = "closed2";
			commandPantryDoor.obj_id = 1;
			LCM.getSingleton().publish("SET_STATE_COMMAND", commandPantryDoor);
		}
		
		System.out.println("Held object is " + held);
		System.out.println("Object id: "+ objectId +"; State vector: " + "stove-door|" +stoveDoorState + " " + "stove-heat|"+ stoveHeatState + " " + "pantry-door|" + pantryDoorState + " " + "arm-state|" + armState + "object-state|" + objectCookedState);
	}
}
