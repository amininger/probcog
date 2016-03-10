package probcog.rosie.actuation;

import java.io.IOException;
import java.util.Properties;

import edu.umich.rosie.soar.AgentConnector;
import edu.umich.rosie.soar.SoarAgent;
import edu.umich.rosie.soar.SoarUtil;
import april.jmat.LinAlg;
import april.util.TimeUtil;
import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import probcog.commands.CommandCoordinator.Status;
import probcog.commands.TypedValue;
import probcog.lcmtypes.control_law_status_t;
import probcog.lcmtypes.control_law_t;
import probcog.lcmtypes.typed_value_t;
import sml.Identifier;

import javax.swing.JMenuBar;

public class MobileActuationConnector extends AgentConnector implements LCMSubscriber{
	private static int CL_FPS = 10;

	private Object commandLock = new Object();
	private control_law_t activeCommand = null;
	private Identifier activeCommandId = null;
	private String commandStatus = "waiting";
	private int nextControlLawId = 1;

	private String movingState = "stopped";

    private LCM lcm;
    
    private boolean killThread = false;
    private ControlLawThread sendCommandThread = null;
    
    public MobileActuationConnector(SoarAgent agent, Properties props){
    	super(agent);

        lcm = LCM.getSingleton();
        
        // Setup Output Link Events
        String[] outputHandlerStrings = { "do-control-law", "stop", "face-point", "pick-up", "put-down"};
        this.setOutputHandlerNames(outputHandlerStrings);

        activeCommand = SoarCommandParser.createEmptyControlLaw("RESTART");
        activeCommand.id = nextControlLawId++;
    }
    
    @Override
    public void connect(){
    	super.connect();
        lcm.subscribe("SOAR_COMMAND_STATUS.*", this);
        killThread = false;
        sendCommandThread = new ControlLawThread();
        sendCommandThread.start();
    }
    
    @Override
    public void disconnect(){
    	super.disconnect();
        lcm.unsubscribe("SOAR_COMMAND_STATUS.*", this);
        if(sendCommandThread != null){
        	killThread = true;
        	try {
				sendCommandThread.join();
			} catch (InterruptedException e) {
				e.printStackTrace();
			}
        	sendCommandThread = null;
        }
    }
    
    public String getMovingState(){
    	return movingState;
    }

	public void createMenu(JMenuBar menuBar) { }

    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
		try {
			if(channel.startsWith("SOAR_COMMAND_STATUS")){
				control_law_status_t newStatus = new control_law_status_t(ins);
				handleCommandStatusMessage(newStatus);
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    
    private void handleCommandStatusMessage(control_law_status_t status){
		if(activeCommand != null && activeCommand.id == status.id){
			Status newStatus = Status.valueOf(status.status);
			if(newStatus == Status.FAILURE || newStatus == Status.SUCCESS || newStatus == Status.UNKNOWN){
				activeCommand = SoarCommandParser.createEmptyControlLaw("NONE");
				activeCommand.id = nextControlLawId++;
				movingState = "stopped";
			}
			commandStatus = newStatus.toString().toLowerCase();
		}
    }

    
    /*****************************************************
     * 
     * ControlLawThread
     *   Thread for periodically sending the activeCommand
     *   over LCM to the robot
     *
     *****************************************************/
	class ControlLawThread extends Thread{
		public ControlLawThread(){}

		public void run(){
			while(!killThread) {
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

    /*******************************************************
     * Methods for updating the input link
     ******************************************************/

	protected void onInputPhase(Identifier inputLink) {
		if(activeCommandId != null){
			SoarUtil.updateStringWME(activeCommandId, "status", commandStatus);
			if(commandStatus.equals("success") || commandStatus.equals("failure") || commandStatus.equals("unknown")){
				activeCommandId = null;
			}
		}
	}
	
	protected void onInitSoar() {
		activeCommandId = null;
	}
	
	/*************************************************************
	 * 
	 * Handling Commands on the Soar Output-Link
	 ************************************************************/

	protected void onOutputEvent(String attName, Identifier id) {
		if(attName.equals("do-control-law")){
			processDoControlLawCommand(id);
		} else if(attName.equals("stop")){
			processStopCommand(id);
		} else if(attName.equals("face-point")){
			processFacePoint(id);
		} else if(attName.equals("pick-up")){
			
		} else if(attName.equals("pick-down")){
			
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
    	id.CreateStringWME("status", "sent");
    	synchronized(commandLock){
    		if (activeCommandId != null){
    			SoarUtil.updateStringWME(activeCommandId, "status", "interrupted");
    		}
    		activeCommand = controlLaw;
    		activeCommandId = id;
    		
			commandStatus = "sent";
    		movingState = "moving";
    	}
    }

    public void processStopCommand(Identifier id){
		synchronized(commandLock){
			if(activeCommandId != null){
				SoarUtil.updateStringWME(activeCommandId, "status", "interrupted");
			}
			activeCommand = SoarCommandParser.createEmptyControlLaw("STOP");
			activeCommand.id = nextControlLawId++;
			activeCommandId = id;
			
			commandStatus = "sent";
			movingState = "stopped";
		}
    }
    
    public void processFacePoint(Identifier id){
		synchronized(commandLock){
			if(activeCommandId != null){
				SoarUtil.updateStringWME(activeCommandId, "status", "interrupted");
			}
			double cx = Double.parseDouble(SoarUtil.getValueOfAttribute(id, "cur-x"));
			double cy = Double.parseDouble(SoarUtil.getValueOfAttribute(id, "cur-y"));
			double dx = Double.parseDouble(SoarUtil.getValueOfAttribute(id, "x"));
			double dy = Double.parseDouble(SoarUtil.getValueOfAttribute(id, "y"));
			double yaw = Math.atan2(dy-cy, dx-cx);

			activeCommandId = id;
			activeCommand = SoarCommandParser.createEmptyControlLaw("orient");
			activeCommand.id = nextControlLawId++;
			activeCommand.num_params = 1;
			activeCommand.param_names = new String[]{ "yaw" };
			activeCommand.param_values = new typed_value_t[]{ (new TypedValue(yaw)).toLCM() };
			activeCommand.termination_condition.name = "stabilized";
			
			commandStatus = "sent";
			movingState = "moving";
		}
    }
}
