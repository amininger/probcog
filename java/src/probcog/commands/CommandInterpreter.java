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
    protected int lawID;    // Could be multiples in future
    protected int testID;   // Could be multiples in future

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
        control_law_status_list_t sl = statusCache.get();
        if (sl == null)
            return;

        if(sl.nstatuses == 0 && waitingCommands.size() > 0) {
            control_law_t nextCommand;
            synchronized (commandLock) {
                nextCommand = waitingCommands.poll();
            }

            // Get control law parameters
            Map<String, TypedValue> paramsControl = new HashMap<String, TypedValue>();
            for(int i=0; i<nextCommand.num_params; i++) {
                paramsControl.put(nextCommand.param_names[i],
                                  new TypedValue(nextCommand.param_values[i]));
            }

            // Get test condition parameters
            Map<String, TypedValue> paramsTest = new HashMap<String, TypedValue>();
            for(int i=0; i<nextCommand.termination_condition.num_params; i++) {
                paramsTest.put(nextCommand.termination_condition.param_names[i],
                               new TypedValue(nextCommand.termination_condition.param_values[i]));
            }

            // Attempt to create and register control law and test condition
            try {
                ControlLaw law = clfactory.construct(nextCommand.name, paramsControl);
                ConditionTest test = ctfactory.construct(nextCommand.termination_condition.name,
                                                         paramsTest);
                if (law != null && test != null) {
                    testID = coordinator.registerConditionTest(test);
                    lawID = coordinator.registerControlLaw(law);
                    coordinator.registerTerminationCondition(testID, lawID, CommandCoordinator.Status.SUCCESS);
                } else {
                    System.err.println("WRN: Error constructing law/test");
                }
            } catch (ClassNotFoundException ex) {
                System.err.println("ERR: "+ex);
                ex.printStackTrace();
            }
        }
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
			if (channel.startsWith("SOAR_COMMAND")) {
				control_law_t controlLaw = new control_law_t(ins);
				newCommand(controlLaw);
			} else if (channel.startsWith("CONTROL_LAW_STATUS")) {
                control_law_status_list_t sl = new control_law_status_list_t(ins);
                statusCache.put(sl, sl.utime);

                // Cleanup
                synchronized (commandLock) {
                    if (sl.nstatuses > 0 && sl.statuses[0].id == lawID)
                    {
                        String status = sl.statuses[0].status;
                        if (!CommandCoordinator.Status.EXECUTING.name().equals(status)) {
                            System.out.println("STATUS == "+status);
                            coordinator.destroyConditionTest(testID);
                            coordinator.destroyControlLaw(lawID);
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
