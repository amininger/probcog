package probcog.rosie;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Properties;

import javax.swing.JButton;
import javax.swing.JMenu;
import javax.swing.JMenuBar;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import probcog.arm.ArmStatus;
import probcog.lcmtypes.robot_action_t;
import probcog.lcmtypes.robot_command_t;
import probcog.lcmtypes.set_state_command_t;
import probcog.rosie.scripting.ResetRobotArm;
import sml.Agent;
import sml.Agent.OutputEventInterface;
import sml.Agent.RunEventInterface;
import sml.Identifier;
import sml.WMElement;
import sml.smlRunEventId;
import april.config.Config;
import april.config.ConfigFile;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.util.TimeUtil;
import edu.umich.rosie.*;
import edu.umich.rosie.soar.*;
import edu.umich.rosie.soarobjects.Pose;



public class ArmActuationConnector extends AgentConnector implements LCMSubscriber{
	private Identifier selfId;
	private Identifier armId;

	private Pose pose;
	
	private robot_action_t curStatus = null;
	private robot_action_t prevStatus = null;
	// Last received information about the arm
	
	private boolean gotUpdate = false;
	private boolean gotArmUpdate = false;
	
    private LCM lcm;
    
    private ArmStatus armStatus;
    
    StringBuilder svsCommands = new StringBuilder();

	private Integer heldObject;
    
    private robot_command_t sentCommand = null;
    private long sentTime = 0;
    
    private Identifier waitingCommandId = null;

    public ArmActuationConnector(SoarAgent agent, Properties props){
    	super(agent);
    	pose = new Pose();
    	
    	String armConfigFilepath = props.getProperty("arm-config", null);
    	if(armConfigFilepath == null){
    		armStatus = null;
    	} else {
	        try {
	  			Config config = new ConfigFile(armConfigFilepath);
	  			armStatus = new ArmStatus(config);
	  		} catch (IOException e) {
	  			armStatus = null;
	  		}
    	}
    	heldObject = -1;

    	// Setup LCM events
        lcm = LCM.getSingleton();
        
        // Setup Output Link Events
        String[] outputHandlerStrings = { "pick-up", "put-down", "point", "set-state", "home", "reset"};
        this.setOutputHandlerNames(outputHandlerStrings);
    }
    
    @Override
    public void connect(){
    	super.connect();
        lcm.subscribe("ROBOT_ACTION", this);
        lcm.subscribe("ARM_STATUS", this);
    }
    
    @Override
    public void disconnect(){
    	super.disconnect();
    	pose.removeFromWM();
        lcm.unsubscribe("ROBOT_ACTION", this);
        lcm.unsubscribe("ARM_STATUS", this);
    }
    
    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
    	if(channel.equals("ROBOT_ACTION")){
    		try {
    			robot_action_t action = new robot_action_t(ins);
				newRobotStatus(action);
			} catch (IOException e) {
				e.printStackTrace();
			}
    	} else if(channel.equals("ARM_STATUS")){
    		gotArmUpdate = true;
    	}
    }
    
    public void newRobotStatus(robot_action_t status){
    	curStatus = status;
    	gotUpdate = true;
    }
    
    public String getStatus(){
    	if(curStatus == null){
    		return "wait";
    	} else {
    		return curStatus.action.toLowerCase();
    	}
    }

	// Happens during an input phase
    @Override
    protected void onInputPhase(Identifier inputLink){
		if(selfId == null){
			initIL();
		} else if(gotUpdate){
			updateOL();
			updateIL();
			gotUpdate = false;
			prevStatus = curStatus;
		}
		if(armStatus != null){
			updateArmInfo();
		}
		if(svsCommands.length() > 0){
			soarAgent.getAgent().SendSVSInput(svsCommands.toString());
			//System.out.println(svsCommands.toString());
			svsCommands = new StringBuilder();
		}
    	if(sentCommand != null && curStatus != null){
    		if(sentCommand.action.toLowerCase().contains("drop")){
    			if(curStatus.action.toLowerCase().equals("drop")){
    				sentCommand = null;
    			} else if(TimeUtil.utime() > sentTime + 2000000){
    		    	lcm.publish("ROBOT_COMMAND", sentCommand);
    		    	sentTime = TimeUtil.utime();
    			}
    		} else if(sentCommand.action.toLowerCase().contains("grab")){
    			if(curStatus.action.toLowerCase().equals("grab")){
    				sentCommand = null;
    			} else if(TimeUtil.utime() > sentTime + 2000000){
    		    	lcm.publish("ROBOT_COMMAND", sentCommand);
    		    	sentTime = TimeUtil.utime();
    			}
    			
    		}
    	}
	}
    
    private void initIL(){
    	selfId = soarAgent.getAgent().GetInputLink().CreateIdWME("self");
    	selfId.CreateStringWME("moving-status", "stopped");

    	armId = selfId.CreateIdWME("arm");
    	armId.CreateStringWME("moving-status", "wait");
    	armId.CreateStringWME("holding-object", "none");

    	pose.updateWithArray(new double[]{0, 0, 0, 0, 0, 0});
    	pose.addToWM(selfId);
    	
    	if(armStatus != null){
        	svsCommands.append("add arm world p 0 0 0 r 0 0 0\n");
        	
        	ArrayList<Double> widths = armStatus.getArmSegmentWidths();
        	ArrayList<double[]> points = armStatus.getArmPoints();
        	for(int i = 0; i < widths.size(); i++){
        		// For each segment on the arm, initialize with the correct bounding volume
        		String name = "seg" + i;
        		
        		double[] p1 = points.get(i);
        		double[] p2 = points.get(i+1);
        		double len = LinAlg.distance(p1, p2); 
        		double[] size = new double[]{len, widths.get(i), widths.get(i)};
        		if(i == widths.size()-1){
        			// Make the gripper bigger to help with occlusion checks;
        			size = LinAlg.scale(size, 2);
        		}
        		
        		svsCommands.append("add " + name + " arm p 0 0 0 r 0 0 0 ");
        		svsCommands.append("s " + size[0] + " " + size[1] + " " + size[2] + " ");
        		svsCommands.append("v " + SVSCommands.bboxVertices() + "\n");
        	}
    	}
    }
    

    private void updateOL(){
    	if(curStatus == null || prevStatus == null || waitingCommandId == null){
    		return;
    	}
    	String curAction = curStatus.action.toLowerCase();
    	String prevAction = prevStatus.action.toLowerCase();
    	if(!prevAction.contains("wait") && !prevAction.contains("failure")){
    		if(curAction.contains("wait")){
				SoarUtil.updateStringWME(waitingCommandId, "status", "success");
    			waitingCommandId = null;
    		} else if(curAction.contains("failure")){
				SoarUtil.updateStringWME(waitingCommandId, "status", "failure");
    			waitingCommandId = null;
    		}
    	}
    }
    
    private void updateIL(){   	
    	heldObject = curStatus.obj_id;
    	SoarUtil.updateStringWME(armId, "moving-status", curStatus.action.toLowerCase());
//    	if(prevStatus == null){
//        	SoarUtil.updateStringWME(selfId, "prev-action", "wait");
//    	} else {
//        	SoarUtil.updateStringWME(selfId, "prev-action", prevStatus.action.toLowerCase());
//    	}
    	ArmPerceptionConnector perception = (ArmPerceptionConnector)soarAgent.getPerceptionConnector();
    	if (curStatus.obj_id == -1){
    		SoarUtil.updateStringWME(armId, "holding-object", "none");
    	} else {
    		SoarUtil.updateStringWME(armId, "holding-object", perception.getWorld().getSoarHandle(curStatus.obj_id).toString());
    	}
    	pose.updateWithArray(curStatus.xyz);
    	pose.updateWM();
    }
    
    private void updateArmInfo(){
    	if(!gotArmUpdate){
    		return;
    	}
    	gotArmUpdate = false;
    	ArrayList<Double> widths = armStatus.getArmSegmentWidths();
    	ArrayList<double[]> points = armStatus.getArmPoints();
    	for(int i = 0; i < widths.size(); i++){
    		String name = "seg" + i;
    		
    		double[] p1 = points.get(i);
			double[] p2 = points.get(i+1);
			double[] center = LinAlg.scale(LinAlg.add(p1, p2), .5);
			double[] dir = LinAlg.subtract(p2, p1);
			
			double hyp = Math.sqrt(dir[0] * dir[0] + dir[1] * dir[1]);
			
			double theta = 0;
			if(Math.abs(dir[0]) > .0001 || Math.abs(dir[1]) > .0001){
				theta = Math.atan2(dir[1], dir[0]);
			} 
			
			double phi = Math.PI/2;
			if(Math.abs(hyp) > .0001 || Math.abs(dir[2]) > .0001){
				phi = -Math.atan2(dir[2], hyp);
			}
			
			double[][] rotZ = LinAlg.rotateZ(theta);
			double[][] rotY = LinAlg.rotateY(phi);
			
			double[] rot = LinAlg.matrixToRollPitchYaw(LinAlg.matrixAB(rotZ, rotY));
			
			svsCommands.append(SVSCommands.changePos(name, center));
			svsCommands.append(SVSCommands.changeRot(name, rot));
    	}
    }
    
    /**********************************************************
     * OUTPUT EVENTS
     ***********************************************************/

    @Override
    protected void onOutputEvent(String attName, Identifier id){
        if (attName.equals("set-state")) {
            processSetCommand(id);
        } 
        else if (attName.equals("pick-up")) {
            processPickUpCommand(id);
        } 
        else if (attName.equals("put-down")) {
            processPutDownCommand(id);
        } 
        else if (attName.equals("point")) {
            processPointCommand(id);
        } 
        else if(attName.equals("home")){
        	processHomeCommand(id);
        }
        else if(attName.equals("reset")){
        	processResetCommand(id);
        }
	}
    
    /**
     * Takes a pick-up command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command. Expects pick-up
     * ^object-id [int]
     */
    private void processPickUpCommand(Identifier pickUpId)
    {
        String objectHandleStr = SoarUtil.getValueOfAttribute(pickUpId,
                "object-handle", "pick-up does not have an ^object-id attribute");
        ArmPerceptionConnector perception = (ArmPerceptionConnector)soarAgent.getPerceptionConnector();
        Integer percId = perception.getWorld().getPerceptionId(Integer.parseInt(objectHandleStr));
        if(percId == null){
        	System.err.println("Pick up: unknown id " + objectHandleStr);
			SoarUtil.updateStringWME(pickUpId, "status", "error");
        	return;
        }
        
        robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime(); 
        command.action = String.format("GRAB=%d", percId);
        command.dest = new double[6];
        System.out.println("PICK UP: " + percId + " (Soar Handle: " + objectHandleStr + ")");
    	lcm.publish("ROBOT_COMMAND", command);
        sentCommand = command;
        sentTime = TimeUtil.utime();
        waitingCommandId = pickUpId;
		SoarUtil.updateStringWME(pickUpId, "status", "sent");
    }

    /**
     * Takes a put-down command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command Expects put-down
     * ^location <loc> <loc> ^x [float] ^y [float] ^z [float]
     */
    private void processPutDownCommand(Identifier putDownId)
    {
        Identifier locationId = SoarUtil.getIdentifierOfAttribute(
                putDownId, "location",
                "Error (put-down): No ^location identifier");
        
        double x = Double.parseDouble(SoarUtil.getValueOfAttribute(
                locationId, "x", "Error (put-down): No ^location.x attribute"));
        double y = Double.parseDouble(SoarUtil.getValueOfAttribute(
                locationId, "y", "Error (put-down): No ^location.y attribute"));
        double z = Double.parseDouble(SoarUtil.getValueOfAttribute(
                locationId, "z", "Error (put-down): No ^location.z attribute"));
        robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime(); 
        command.action = "DROP";
        command.dest = new double[]{x, y, z, 0, 0, 0};
    	lcm.publish("ROBOT_COMMAND", command);
        sentCommand = command;
        sentTime = TimeUtil.utime();
        System.out.println("Put down at " + x + ", " + y + ", " + z);
        waitingCommandId = putDownId;
		SoarUtil.updateStringWME(putDownId, "status", "sent");
    }

    /**
     * Takes a set-state command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command
     */
    private void processSetCommand(Identifier id)
    {
        String objHandleStr = SoarUtil.getValueOfAttribute(id, "object-handle",
                "Error (set-state): No ^object-handle attribute");
        ArmPerceptionConnector perception = (ArmPerceptionConnector)soarAgent.getPerceptionConnector();
        Integer percId = perception.getWorld().getPerceptionId(Integer.parseInt(objHandleStr));
        if(percId == null){
        	System.err.println("Set: unknown id " + objHandleStr);
			SoarUtil.updateStringWME(id, "status", "error");
        	return;
        }
        
        String name = SoarUtil.getValueOfAttribute(id,
                "name", "Error (set-state): No ^name attribute");
        String value = SoarUtil.getValueOfAttribute(id, "value",
                "Error (set-state): No ^value attribute");

        set_state_command_t command = new set_state_command_t();
        command.utime = TimeUtil.utime(); 
        command.state_name = name;
        command.state_val = value;
        command.obj_id = percId;
    	lcm.publish("SET_STATE_COMMAND", command);
		SoarUtil.updateStringWME(id, "status", "sent");
    }

    private void processPointCommand(Identifier pointId)
    {
    	String objHandleStr = SoarUtil.getValueOfAttribute(pointId, "object-handle", 
    			"Error (point): No ^object-handle attribute");
    	ArmPerceptionConnector perc = (ArmPerceptionConnector)soarAgent.getPerceptionConnector();
        Integer percId = perc.getWorld().getPerceptionId(Integer.parseInt(objHandleStr));
        if(percId == null){
        	System.err.println("Set: unknown handle " + objHandleStr);
			SoarUtil.updateStringWME(pointId, "status", "error");
        	return;
        }
        
        robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime(); 
        command.dest = new double[]{0, 0, 0, 0, 0, 0};
    	command.action = "POINT=" + percId;
    	lcm.publish("ROBOT_COMMAND", command);
		SoarUtil.updateStringWME(pointId, "status", "sent");
    }
    
    private void processHomeCommand(Identifier id){
    	robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime(); 
        command.dest = new double[6];
    	command.action = "HOME";
    	lcm.publish("ROBOT_COMMAND", command);
		SoarUtil.updateStringWME(id, "status", "sent");
    }

    private void processResetCommand(Identifier id){
    	robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime(); 
        command.dest = new double[6];
    	command.action = "RESET";
    	lcm.publish("ROBOT_COMMAND", command);
		SoarUtil.updateStringWME(id, "status", "sent");
    }
    
    public void createMenu(JMenuBar menuBar){
    	JMenu actionMenu = new JMenu("Action");
    	JButton armResetButton  = new JButton("Reset Arm");
        armResetButton.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				new ResetRobotArm().execute();
			}
        });
        actionMenu.add(armResetButton);
        
        menuBar.add(actionMenu);
    }

	public Integer getHeldObject() {
		return heldObject;
	}

	@Override
	protected void onInitSoar() {
		pose.removeFromWM();
		selfId = null;
		armId = null;
		waitingCommandId = null;
	}
}
