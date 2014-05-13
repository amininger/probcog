package probcog.commands;

import lcm.lcm.*;

import java.util.*;
import java.io.IOException;

import april.util.*;

import probcog.lcmtypes.*;

import probcog.commands.controls.*;
import probcog.commands.tests.*;


public class CommandInterpreter
{
	protected static LCM lcm = LCM.getSingleton();
	private static int LCM_FPS = 60;
	private static int CMD_FPS = 20;

    CommandCoordinator coordinator = new CommandCoordinator();
    ControlLawFactory clfactory = ControlLawFactory.getSingleton();
    ConditionTestFactory ctfactory = ConditionTestFactory.getSingleton();

	protected Object commandLock = new Object();

	protected ControlLaw curCommand;

	protected Queue<control_law_t> waitingCommands;

    protected ExpiringMessageCache<control_law_status_list_t> statusCache =
        new ExpiringMessageCache<control_law_status_list_t>(.25);

	public CommandInterpreter()
    {
		waitingCommands = new LinkedList<control_law_t>();

		new ListenerThread().start();
		new CommandThread().start();
	}

	protected void newCommand(control_law_t controlLaw)
    {
		synchronized(commandLock) {
			waitingCommands.add(controlLaw);
		}
	}

    // XXX Deprecated. Slated for removal
	protected void sendStatus(int clId, String status)
    {
		control_law_status_t clStatus = new control_law_status_t();
		clStatus.id = clId;
		clStatus.status = status;
		lcm.publish("SOAR_COMMAND_STATUS", clStatus);
	}

	protected void update()
    {
		synchronized(commandLock){
            control_law_status_list_t sl = statusCache.get();
            if (sl == null)
                return;

			if(sl.nstatuses == 0 && waitingCommands.size() > 0) {
				control_law_t nextCommand = waitingCommands.poll();
                assert (nextCommand.name.equals("drive-forward") ||
                        nextCommand.name.equals("turn"));
                assert (nextCommand.termination_condition.name.equals("distance") ||
                        nextCommand.termination_condition.name.equals("rotation"));
                Map<String, TypedValue> params = new HashMap<String, TypedValue>();
                if (nextCommand.name.equals("turn")) {
                    int v = TypedValue.unwrapInt(nextCommand.param_values[0]);
                    params.put("direction", new TypedValue((byte)v));
                }
                Map<String, TypedValue> params2 = new HashMap<String, TypedValue>();
                double v = TypedValue.unwrapDouble(nextCommand.termination_condition.compared_value);
                if (nextCommand.termination_condition.name.equals("distance")) {
                    params2.put("distance", new TypedValue(v));
                } else {
                    params2.put("yaw", new TypedValue(v));
                }

                try {
                    ControlLaw law = clfactory.construct(nextCommand.name, params);
                    ConditionTest test = ctfactory.construct(nextCommand.termination_condition.name, params2);
                    if (law != null && test != null) {
                        int ctid = coordinator.registerConditionTest(test);
                        int clid = coordinator.registerControlLaw(law);
                        coordinator.registerTerminationCondition(ctid, clid, CommandCoordinator.Status.SUCCESS);
                    } else {
                        System.err.println("WRN: Error constructing law/test");
                    }
                    /*if(curCommand == null){
                        sendStatus(nextCommand.id, "unknown-command");
                    } else {
                        sendStatus(curCommand.getID(), "started");
                    }*/
                } catch (ClassNotFoundException ex) {
                    System.err.println("ERR: "+ex);
                    ex.printStackTrace();
                }
			}
		}

		/*if(curCommand != null) {
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
		}*/
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
			lcm.subscribe("SOAR_COMMAND", this);
            lcm.subscribe("CONTROL_LAW_STATUS", this);
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
			if (channel.equals("SOAR_COMMAND")) {
				control_law_t controlLaw = new control_law_t(ins);
				newCommand(controlLaw);
			} else if ("CONTROL_LAW_STATUS".equals(channel)) {
                control_law_status_list_t sl = new control_law_status_list_t(ins);
                statusCache.put(sl, TimeUtil.utime());
            }
		}
	}

	public static void main(String[] args)
    {
		CommandInterpreter ci = new CommandInterpreter();
	}
}
