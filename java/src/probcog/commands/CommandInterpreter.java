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
		public Status status;
		public boolean running;
		public CommandInfo(int lawID_, int testID_, int commandID_, Status status_){
			lawID = lawID_; 
			testID = testID_; 
			commandID = commandID_;
			setStatus(status_);
		}
		public void setStatus(Status newStatus){
			running = (newStatus == Status.EXECUTING || newStatus == Status.RECEIVED);
			status = newStatus;
		}
	}
	
	protected static LCM lcm = LCM.getSingleton();
	private static int LCM_FPS = 60;
	private static int CMD_FPS = 20;

    CommandCoordinator coordinator = new CommandCoordinator();
    ControlLawFactory clfactory = ControlLawFactory.getSingleton();
    ConditionTestFactory ctfactory = ConditionTestFactory.getSingleton();

	protected Object commandLock = new Object();
	protected CommandInfo activeCommand = null;
	protected int highestCommandID = -1;

    // Handle stale/repeat control law messages
    //protected int lastCommandID = -1;
	//protected Queue<control_law_t> waitingCommands;

    protected ExpiringMessageCache<control_law_status_list_t> statusCache =
        new ExpiringMessageCache<control_law_status_list_t>(.25);

	public CommandInterpreter()
    {
		//waitingCommands = new LinkedList<control_law_t>();

		new ListenerThread().start();
		new CommandThread().start();
	}

	protected void interpretCommand(control_law_t controlLaw)
    {
		synchronized(commandLock){
			if(activeCommand != null && controlLaw.id <= highestCommandID){
				return;
			}
			highestCommandID = controlLaw.id;
	        if(controlLaw.name.toLowerCase().equals("none")){
	        	// Control Law Name == NONE
	        	// Stop current command and destroy it
	        	if (activeCommand != null){
	        		stopActiveCommand();
	        		activeCommand = null;
	        	}
	        } else if(controlLaw.name.toLowerCase().equals("stop")){
	        	// Control Law Name == STOP
	        	// Stop current command
	        	if (activeCommand != null && activeCommand.running){
	        		stopActiveCommand();
	        		activeCommand = new CommandInfo(-1, -1, controlLaw.id, Status.SUCCESS);
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
	                activeCommand = new CommandInfo(lawID, testID, newCommand.id, Status.RECEIVED);
	            } else {
	                System.err.println("WRN: Error constructing law/test");
	                activeCommand = new CommandInfo(-1, -1, newCommand.id, Status.FAILURE);
	            }
	        } catch (ClassNotFoundException ex) {
	            System.err.println("ERR: "+ex);
	            ex.printStackTrace();
	            activeCommand = new CommandInfo(-1, -1, newCommand.id, Status.FAILURE);
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

	protected void sendCommandStatus(control_law_t controlLaw)
    {
		if(controlLaw.name.toLowerCase().equals("none")){
			// No commands being sent, no need to reply with a status
			return;
		}
		if(activeCommand == null){
			// No command to report on
			return;
		}
		control_law_status_t clStatus = new control_law_status_t();
		clStatus.id = activeCommand.commandID;
		clStatus.status = activeCommand.status.toString();
		clStatus.name = controlLaw.name;
		lcm.publish("SOAR_COMMAND_STATUS", clStatus);
	}

	protected void update()
    {
		// AM: Removed because for now we only assume 1 control law at a time, 
		//     so there is no queuing, if a new one arrives we kill the old one
//        control_law_status_list_t sl = statusCache.get();
//        if (sl == null)
//            return;
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
				sendCommandStatus(controlLaw);
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
