package probcog.commands;

import lcm.lcm.*;

import java.util.*;
import java.io.IOException;

import april.util.*;

import probcog.lcmtypes.*;

import probcog.commands.CommandCoordinator.Status;
import probcog.commands.controls.*;
import probcog.commands.tests.*;


public class CommandInterpreter
{
	protected class CommandInfo{
		public int lawID;
		public int testID;
		public int commandID;
        public String name;
		public Status status;
		public boolean running;
		public CommandInfo(int lawID_, int testID_, int commandID_, String name_, Status status_){
			lawID = lawID_;
			testID = testID_;
			commandID = commandID_;
            name = name_;
			setStatus(status_);
		}
		public void setStatus(Status newStatus){
			running = (newStatus == Status.EXECUTING || newStatus == Status.RECEIVED);
			status = newStatus;
		}
	}

    private boolean running;
	protected static LCM lcm = LCM.getSingleton();
	private static int LCM_FPS = 60;
	private static int CMD_FPS = 20;

	CommandCoordinator coordinator = new CommandCoordinator();
	ControlLawFactory clfactory = ControlLawFactory.getSingleton();
	ConditionTestFactory ctfactory = ConditionTestFactory.getSingleton();

	protected Object commandLock = new Object();
	protected CommandInfo activeCommand = null;
	protected int highestCommandID = -1;

    // Is this in a sim robot?
    boolean sim;

	// Handle stale/repeat control law messages
	//protected int lastCommandID = -1;
	//protected Queue<control_law_t> waitingCommands;

	protected ExpiringMessageCache<control_law_status_list_t> statusCache =
		new ExpiringMessageCache<control_law_status_list_t>(.25);

    public CommandInterpreter()
    {
        this(false);
    }

	public CommandInterpreter(boolean sim)
	{
		//waitingCommands = new LinkedList<control_law_t>();
        this.sim = sim;

        running = true;
		new ListenerThread().start();
		new CommandThread().start();
	}

    public void setRunning(boolean run)
    {
        running = run;
        coordinator.setRunning(run);
    }

	protected void interpretCommand(control_law_t controlLaw)
	{
		synchronized(commandLock){
			String clName = controlLaw.name.toLowerCase();
			if(clName.equals("restart")){
				// Control Law Name == RESTART
				/// Stop current command and destory, and reset counter
				if(activeCommand != null){
					stopActiveCommand();
					activeCommand = null;
				}
				highestCommandID = controlLaw.id;
			}
			if(controlLaw.id <= highestCommandID){
				return;
			}
			highestCommandID = controlLaw.id;
			if(clName.equals("none")){
				// Control Law Name == NONE
				// Stop current command and destroy it
				if (activeCommand != null){
					stopActiveCommand();
					activeCommand = null;
				}
			} else if(clName.equals("stop")){
				// Control Law Name == STOP
				// Stop current command
				if (activeCommand != null && activeCommand.running){
					stopActiveCommand();
					activeCommand = new CommandInfo(-1, -1, controlLaw.id, "stop", Status.SUCCESS);
				}
			} else {
				// Other Control Law
				executeNewCommand(controlLaw);
			}
		}
	}

	protected void executeNewCommand(control_law_t newCommand){
		synchronized(commandLock){
			stopActiveCommand();
			activeCommand = null;

			// Get control law parameters
			Map<String, TypedValue> paramsControl = new HashMap<String, TypedValue>();
			for(int i=0; i<newCommand.num_params; i++) {
				paramsControl.put(newCommand.param_names[i],
								  new TypedValue(newCommand.param_values[i]));
			}

            if (sim) {
                paramsControl.put("sim", new TypedValue(1));
            }

			// Get test condition parameters
			Map<String, TypedValue> paramsTest = new HashMap<String, TypedValue>();
			for(int i=0; i<newCommand.termination_condition.num_params; i++) {
				paramsTest.put(newCommand.termination_condition.param_names[i],
							   new TypedValue(newCommand.termination_condition.param_values[i]));
			}

			// Attempt to create and register control law and test condition
			try {
				ControlLaw law = clfactory.construct(newCommand.name, paramsControl);
				ConditionTest test = ctfactory.construct(newCommand.termination_condition.name,
														 paramsTest);
				if (law != null && test != null) {
					int testID = coordinator.registerConditionTest(test);
					int lawID = coordinator.registerControlLaw(law);
					coordinator.registerTerminationCondition(testID, lawID, CommandCoordinator.Status.SUCCESS);
					activeCommand = new CommandInfo(lawID, testID, newCommand.id, law.getName(), Status.RECEIVED);
				} else {
					System.err.println("WRN: Error constructing law/test");
					activeCommand = new CommandInfo(-1, -1, newCommand.id,"NULL", Status.FAILURE);
				}
			} catch (ClassNotFoundException ex) {
				System.err.println("ERR: "+ex);
				ex.printStackTrace();
				activeCommand = new CommandInfo(-1, -1, newCommand.id, "ERR", Status.FAILURE);
			}
		}
	}

	protected void stopActiveCommand(){
		synchronized(commandLock){
			if (activeCommand == null){
				return;
			}
			if(activeCommand.testID != -1){
				coordinator.destroyConditionTest(activeCommand.testID);
				activeCommand.testID = -1;
			}
			if(activeCommand.lawID != -1){
				coordinator.destroyControlLaw(activeCommand.lawID);
				activeCommand.lawID = -1;
			}
		}
	}

	protected void sendCommandStatus()
	{
        if (activeCommand != null) {
            String clName = activeCommand.name.toLowerCase();
            if(clName.equals("none") || clName.equals("restart")){
                // No commands being sent, no need to reply with a status
                return;
            }
        }
		control_law_status_t clStatus = new control_law_status_t();
        clStatus.utime = TimeUtil.utime();
        clStatus.id = 0;
        clStatus.status = Status.SUCCESS.toString();
        clStatus.name = "stop";

		if(activeCommand == null){
			// No command to report on
            lcm.publish("SOAR_COMMAND_STATUS_TX", clStatus);
			return;
		}

		clStatus.id = activeCommand.commandID;
		clStatus.status = activeCommand.status.toString();
		clStatus.name = activeCommand.name;
		lcm.publish("SOAR_COMMAND_STATUS_TX", clStatus);
	}

	protected void update()
	{
		// AM: Removed because for now we only assume 1 control law at a time,
		//	 so there is no queuing, if a new one arrives we kill the old one
//		control_law_status_list_t sl = statusCache.get();
//		if (sl == null)
//			return;
	}

	class CommandThread extends Thread
	{
		public CommandThread()
		{
		}

		public void run()
		{
			while(true) {
				update();
				TimeUtil.sleep(1000/CMD_FPS);
			}
		}
	}

	class ListenerThread extends Thread implements LCMSubscriber
	{
		LCM lcm = LCM.getSingleton();

		public ListenerThread()
		{
			lcm.subscribe("SOAR_COMMAND.*", this);
			lcm.subscribe("CONTROL_LAW_STATUS.*", this);
		}

		public void run()
		{
			while(true) {
				TimeUtil.sleep(1000/LCM_FPS);
			}
		}

		public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
		{
            if (!running)
                return;

			try {
				messageReceivedEx(lcm, channel, ins);
			} catch (IOException ioex) {
				System.err.println("ERR: LCM channel " + channel);
				ioex.printStackTrace();
			}
		}

		public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
			throws IOException
		{
			if (channel.startsWith("SOAR_COMMAND") && !channel.startsWith("SOAR_COMMAND_STATUS")) {
				control_law_t controlLaw = new control_law_t(ins);
				interpretCommand(controlLaw);
				sendCommandStatus();
			} else if (channel.startsWith("CONTROL_LAW_STATUS")) {
				synchronized(commandLock){
					control_law_status_list_t sl = new control_law_status_list_t(ins);
					statusCache.put(sl, sl.utime);
					for (int i = 0; i < sl.nstatuses; i++){
						if (activeCommand != null && sl.statuses[i].id == activeCommand.lawID){
							Status newStatus = Status.valueOf(sl.statuses[i].status);
							if(activeCommand.running && newStatus != Status.EXECUTING){
								stopActiveCommand();
							}
							activeCommand.setStatus(newStatus);
                            sendCommandStatus();
						}
					}
				}
			}
		}
	}

	public static void main(String[] args)
	{
		CommandInterpreter ci = new CommandInterpreter();
	}
}
