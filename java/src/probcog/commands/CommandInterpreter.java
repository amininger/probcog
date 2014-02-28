package probcog.commands;

import lcm.lcm.*;

import java.util.Queue;
import java.util.LinkedList;
import java.io.IOException;

import april.util.TimeUtil;

import probcog.lcmtypes.control_law_t;
import probcog.lcmtypes.control_law_status_t;

import probcog.commands.controls.ControlLaw;
import probcog.commands.controls.ControlLawFactory;


public class CommandInterpreter{
	protected static LCM lcm = LCM.getSingleton();
	private static int LCM_FPS = 60;
	private static int CMD_FPS = 20;

	protected Object commandLock = new Object();

	protected ControlLaw curCommand;

	protected Queue<control_law_t> waitingCommands;

	public CommandInterpreter(){
		waitingCommands = new LinkedList<control_law_t>();

		new ListenerThread().start();
		new CommandThread().start();
	}

	protected void newCommand(control_law_t controlLaw){
		synchronized(commandLock){
            System.out.printf("ADDING CMD\n");
			waitingCommands.add(controlLaw);
		}
	}

	protected void sendStatus(int clId, String status){
		control_law_status_t clStatus = new control_law_status_t();
		clStatus.id = clId;
		clStatus.status = status;
		lcm.publish("SOAR_COMMAND_STATUS", clStatus);
	}

	protected void update(){
		synchronized(commandLock){
			if(curCommand == null && waitingCommands.size() > 0){
				control_law_t nextCommand = waitingCommands.poll();
				curCommand = ControlLawFactory.construct(nextCommand);
                System.out.printf("FOUND COMMAND: %s\n", nextCommand.name);
				if(curCommand == null){
					sendStatus(nextCommand.id, "unknown-command");
				} else {
					sendStatus(curCommand.getID(), "started");
				}
			}
		}

		if(curCommand != null){
            System.out.printf("EXEC: %s\n", curCommand.getName());
			curCommand.execute();
			ControlLaw.Status status = curCommand.getStatus();
			if(status == ControlLaw.Status.FINISHED){
				sendStatus(curCommand.getID(), "complete");
				curCommand = null;
			} else if(status == ControlLaw.Status.EARLY_TERM){
				sendStatus(curCommand.getID(), "early-termination");
				curCommand = null;
			} else if(status == ControlLaw.Status.FAILURE){
				sendStatus(curCommand.getID(), "failure");
				curCommand = null;
			}
		}
	}

	class CommandThread extends Thread{
		public CommandThread(){

		}

		public void run(){
			while(true){
				update();
				TimeUtil.sleep(1000/CMD_FPS);
			}
		}
	}

	class ListenerThread extends Thread implements LCMSubscriber {
		LCM lcm = LCM.getSingleton();

		public ListenerThread(){
			lcm.subscribe("SOAR_COMMAND", this);
		}

		public void run(){
			while(true){
				TimeUtil.sleep(1000/LCM_FPS);
			}
		}

		public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
			try{
				messageReceivedEx(lcm, channel, ins);
			} catch (IOException ioex){
				System.err.println("ERR: LCM channel " + channel);
				ioex.printStackTrace();
			}
		}

		public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException {
			if(channel.equals("SOAR_COMMAND")){
                System.out.printf("GOT COMMAND\n");
				control_law_t controlLaw = new control_law_t(ins);
				newCommand(controlLaw);
			}
		}
	}

	public static void main(String[] args){
		CommandInterpreter ci = new CommandInterpreter();
	}
}
