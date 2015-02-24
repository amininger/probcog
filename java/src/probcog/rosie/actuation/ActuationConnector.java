package probcog.rosie.actuation;

import java.io.IOException;
import java.util.HashMap;
import java.util.LinkedList;
import java.util.Queue;

import april.util.TimeUtil;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import probcog.commands.CommandCoordinator;
import probcog.commands.CommandCoordinator.Status;
//import probcog.lcmtypes.control_law_list_t;
import probcog.lcmtypes.control_law_status_list_t;
import probcog.lcmtypes.control_law_status_t;
import probcog.lcmtypes.control_law_t;
import probcog.rosie.SoarAgent;
import probcog.rosie.WMUtil;
import sml.Agent;
import sml.Agent.OutputEventInterface;
import sml.Agent.RunEventInterface;
import sml.Identifier;
import sml.WMElement;
import sml.smlRunEventId;

public class ActuationConnector implements LCMSubscriber, RunEventInterface, OutputEventInterface{
	private static int CL_FPS = 10;

	private SoarAgent agent;
	private Identifier selfId;

	private Object commandLock = new Object();
	private control_law_t activeCommand = null;
	private Identifier activeCommandId = null;
	private int nextControlLawId = 1;

	private boolean gotUpdate = false;

    private LCM lcm;

    StringBuilder svsCommands = new StringBuilder();

    public ActuationConnector(SoarAgent agent){
    	this.agent = agent;

    	// Setup LCM events
        lcm = LCM.getSingleton();
        lcm.subscribe("SOAR_COMMAND_STATUS.*", this);

        // Setup Input Link Events
        agent.getAgent().RegisterForRunEvent(smlRunEventId.smlEVENT_BEFORE_INPUT_PHASE, this, null);

        // Setup Output Link Events
        String[] outputHandlerStrings = { "do-control-law", "stop"};
        for (String outputHandlerString : outputHandlerStrings)
        {
        	agent.getAgent().AddOutputHandler(outputHandlerString, this, null);
        }
        
        activeCommand = SoarCommandParser.createEmptyControlLaw("NONE");
        activeCommand.id = nextControlLawId++;
        
        (new ControlLawThread()).start();
    }

    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
		try {
			if(channel.startsWith("SOAR_COMMAND_STATUS")){
				control_law_status_t status = new control_law_status_t(ins);
				newControlLawStatus(status);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }

	public void newControlLawStatus(control_law_status_t status){
		synchronized(commandLock){
			if(activeCommand != null && activeCommand.id == status.id){
				WMUtil.updateStringWME(activeCommandId, "status", status.status.toLowerCase());
				Status newStatus = Status.valueOf(status.status);
				if(newStatus == Status.FAILURE || newStatus == Status.SUCCESS || newStatus == Status.UNKNOWN){
					activeCommand = SoarCommandParser.createEmptyControlLaw("NONE");
					activeCommand.id = nextControlLawId++;
					activeCommandId = null;
				}
			}
		}
	}

	// Happens during an input phase
	public synchronized void runEventHandler(int eventID, Object data, Agent agent, int phase){
		if(selfId == null){
			initIL();
		} else if(gotUpdate){
			updateIL();
			gotUpdate = false;
		}
	}

    private void initIL(){
    	Identifier inputLink = agent.getAgent().GetInputLink();
    	inputLink.CreateIdWME("self");
    }

    private void updateIL(){
    	Identifier inputLink = agent.getAgent().GetInputLink();
    }

	class ControlLawThread extends Thread{
		public ControlLawThread(){}

		public void run(){
			while(true) {
				synchronized(commandLock){
					if(activeCommand != null){
						activeCommand.utime = TimeUtil.utime();
						lcm.publish("SOAR_COMMAND_TX", activeCommand);
					}
				}

				TimeUtil.sleep(1000/CL_FPS);
			}
		}
	}

    @Override
    public synchronized void outputEventHandler(Object data, String agentName,
            String attributeName, WMElement wme) {
		if (!(wme.IsJustAdded() && wme.IsIdentifier()))
        {
            return;
        }
		Identifier id = wme.ConvertToIdentifier();
        System.out.println(wme.GetAttribute());

        try{
			if(wme.GetAttribute().equals("do-control-law")){
				processDoControlLawCommand(id);
			} else if(wme.GetAttribute().equals("stop")){
				processStopCommand(id);
			}
        } catch (IllegalStateException e){
        	System.out.println(e.getMessage());
        }
    }
    
    public void processDoControlLawCommand(Identifier id){
    	control_law_t controlLaw = SoarCommandParser.parseControlLaw(id);
    	if(controlLaw == null){
    		id.CreateStringWME("status", "error");
    		id.CreateStringWME("error-type", "syntax-error");
    		return;
    	}
    	controlLaw.id = nextControlLawId++;
    	synchronized(commandLock){
    		activeCommand = controlLaw;
    		activeCommandId = id;
    	}
    }
    
    public void processStopCommand(Identifier id){
		synchronized(commandLock){
			if(activeCommandId != null){
				WMUtil.updateStringWME(activeCommandId, "status", "interrupted");
			}
			activeCommand = SoarCommandParser.createEmptyControlLaw("STOP");
			activeCommand.id = nextControlLawId++;
			activeCommandId = id;
		}
    }
}
