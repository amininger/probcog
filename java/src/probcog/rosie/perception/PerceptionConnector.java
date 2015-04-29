package probcog.rosie.perception;

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
import probcog.lcmtypes.classification_list_t;
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

public class PerceptionConnector implements LCMSubscriber, RunEventInterface{
	private SoarAgent agent;

	private Object lock = new Object();

	private boolean gotUpdate = false;
	private classification_list_t curClassifications = null;

    private LCM lcm;

    public PerceptionConnector(SoarAgent agent){
    	this.agent = agent;

    	// Setup LCM events
        lcm = LCM.getSingleton();
        lcm.subscribe("CLASSIFICATIONS.*", this);

        // Setup Input Link Events
        agent.getAgent().RegisterForRunEvent(smlRunEventId.smlEVENT_BEFORE_INPUT_PHASE, this, null);

    }

    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
		try {
			if(channel.startsWith("CLASSIFICATIONS")){
				curClassifications = new classification_list_t(ins);
				gotUpdate = true;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }

	// Happens during an input phase
	public synchronized void runEventHandler(int eventID, Object data, Agent agent, int phase){
		if (gotUpdate){
			updateInputLink(agent.GetInputLink());
			agent.Commit();
			gotUpdate = false;
		}
	}

    private void updateInputLink(Identifier inputLink){

    }
}
