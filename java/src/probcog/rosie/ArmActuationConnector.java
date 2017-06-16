package probcog.rosie;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.util.ArrayList;
import java.util.Properties;

import javax.swing.JButton;
import javax.swing.JMenu;
import javax.swing.JMenuBar;

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
import probcog.rosie.perception.*;

import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.messages.*;
import edu.wpi.rail.jrosbridge.callback.*;
import javax.json.*;

public class ArmActuationConnector extends AgentConnector{
	private Identifier selfId;
	private Identifier armId;

	private boolean gotUpdate = false;
	private boolean gotArmUpdate = false;

    private Ros ros;
    private Topic armCommands;
	private Topic simCommands; // commands to probcog simulator

	// Last received information about the arm
	private JsonObject curStatus = null;
	private JsonObject prevStatus = null;

    private JsonObject sentCommand = null;
    private long sentTime = 0;

    private StringBuilder svsCommands = new StringBuilder();

	private Integer heldObject;

    private Identifier waitingCommandId = null;

	private JsonObject setStateCommand = null;
	private long setStateSentTime = 0;

    public ArmActuationConnector(SoarAgent agent, Properties props){
    	super(agent);

    	String armConfigFilepath = props.getProperty("arm-config", null);
    	heldObject = -1;

        // Setup Output Link Events
        String[] outputHandlerStrings = { "pick-up", "put-down", "point", "set-state", "home", "reset"};
        this.setOutputHandlerNames(outputHandlerStrings);
    }

    @Override
    public void connect(){
    	super.connect();
        ros = new Ros();
        ros.connect();

        if (ros.isConnected()) {
            System.out.println("ArmActuationConnector connected to rosbridge server.");
        }
        else {
            System.out.println("ArmActuationConnector NOT CONNECTED TO ROSBRIDGE");
        }

        armCommands = new Topic(ros,
                                "/rosie_arm_commands",
                                "rosie_msgs/RobotCommand",
                                500);

        simCommands = new Topic(ros,
                                "/rosie_set_state_commands",
                                "rosie_msgs/SetStateCommand",
                                500);

        Topic armAct = new Topic(ros,
                                 "/rosie_arm_status",
                                 "rosie_msgs/RobotAction");
        System.out.println("Subscribing to arm status updates!");
        armAct.subscribe(new TopicCallback() {
                public void handleMessage(Message message) {
                    curStatus = message.toJsonObject();
                    gotUpdate = true;
                }
            });
    }

    @Override
    public void disconnect(){
    	super.disconnect();
    }

	@Override
	protected void onInitSoar() {
		selfId = null;
		armId = null;
		waitingCommandId = null;
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

		if(svsCommands.length() > 0){
			soarAgent.getAgent().SendSVSInput(svsCommands.toString());
			svsCommands = new StringBuilder();
		}
    	if(sentCommand != null && curStatus != null){
    		if(sentCommand.getString("action").toLowerCase().contains("drop")){
    			if(curStatus.getString("action").toLowerCase().contains("drop")){
    				sentCommand = null;
    			} else if(TimeUtil.utime() > sentTime + 2000000){
                    Message m = new Message(sentCommand);
                    armCommands.publish(m);
    		    	sentTime = TimeUtil.utime();
    			}
    		} else if(sentCommand.getString("action").toLowerCase().contains("grab")){
    			if(curStatus.getString("action").toLowerCase().contains("grab")){
    				sentCommand = null;
    			} else if(TimeUtil.utime() > sentTime + 2000000){
                    Message m = new Message(sentCommand);
                    armCommands.publish(m);
                    System.out.println(m.toString());
    		    	sentTime = TimeUtil.utime();
    			}
    		}
    	}
		if(setStateCommand != null){
			ArmPerceptionConnector perception = (ArmPerceptionConnector)soarAgent.getPerceptionConnector();
			Integer objId = perception.getWorld().getSoarHandle(setStateCommand.getInt("objectId"));
			WorldObject obj = perception.getWorld().getObject(objId);
			if(obj == null || obj.hasProperty(setStateCommand.getString("stateName"), setStateCommand.getString("stateValue"))){
				setStateCommand = null;
			} else if(TimeUtil.utime() > setStateSentTime + 2000000){
				Message m = new Message(setStateCommand);
				simCommands.publish(m);
				setStateSentTime = TimeUtil.utime();
			}
		}
	}

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

    /**********************************************************
     * INPUT
     ***********************************************************/

    private void initIL(){
    	selfId = soarAgent.getAgent().GetInputLink().CreateIdWME("self");
    	selfId.CreateStringWME("moving-status", "stopped");

    	armId = selfId.CreateIdWME("arm");
    	armId.CreateStringWME("moving-status", "wait");
    	armId.CreateStringWME("holding-object", "none");
    }

    private void updateIL(){
    	heldObject = curStatus.getInt("obj_id");
    	SoarUtil.updateStringWME(armId, "moving-status",
                                  curStatus.getString("action").toLowerCase());
             if(prevStatus == null){
            SoarUtil.updateStringWME(selfId, "prev-action", "wait");
        } else {
            SoarUtil.updateStringWME(selfId, "prev-action",
                                     prevStatus.getString("action").toLowerCase());
        }
    	ArmPerceptionConnector perception = (ArmPerceptionConnector)soarAgent.getPerceptionConnector();
    	if (curStatus.getInt("obj_id") == -1){
    		SoarUtil.updateStringWME(armId, "holding-object", "none");
    	} else {
    		SoarUtil.updateStringWME(armId, "holding-object",
                                     perception.getWorld().getSoarHandle(curStatus.getInt("obj_id")).toString());
    	}
        JsonObject gripXYZ = curStatus.getJsonObject("gripper_pos").getJsonObject("translation");
        double[] gp = new double[]{gripXYZ.getJsonNumber("x").doubleValue(),
                                   gripXYZ.getJsonNumber("y").doubleValue(),
                                   gripXYZ.getJsonNumber("z").doubleValue()};
    }

    /**********************************************************
     * OUTPUT
     ***********************************************************/

    private void updateOL(){
    	if(curStatus == null || prevStatus == null || waitingCommandId == null){
    		return;
    	}
    	String curAction = curStatus.getString("action").toLowerCase();
    	String prevAction = prevStatus.getString("action").toLowerCase();
    	if(!prevAction.contains("wait") && !prevAction.contains("failure")){
    		if(curAction.contains("wait")){
    			SoarUtil.updateStringWME(waitingCommandId, "status", "complete");
    			waitingCommandId = null;
    		} else if(curAction.contains("failure")){
    			SoarUtil.updateStringWME(waitingCommandId, "status", "failure");
    			waitingCommandId = null;
    		}
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
        	pickUpId.CreateStringWME("status", "error");
        	return;
        }

        JsonObject jo = Json.createObjectBuilder()
            .add("utime", TimeUtil.utime())
            .add("action", String.format("GRAB=%d", percId))
            .build();
        Message m = new Message(jo);
        armCommands.publish(m);

        sentTime = TimeUtil.utime();
        sentCommand = jo;
        waitingCommandId = pickUpId;
		SoarUtil.updateStringWME(pickUpId, "status", "sent");
        System.out.println("PICK UP: " + percId +
                           " (Soar Handle: " + objectHandleStr + ")");
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

        JsonObject jo = Json.createObjectBuilder()
            .add("utime", TimeUtil.utime())
            .add("action", "DROP")
            .add("drop", Json.createObjectBuilder()
                 .add("x", x)
                 .add("y", y)
                 .add("z", z))
            .build();
        Message m = new Message(jo);
        armCommands.publish(m);

        sentTime = TimeUtil.utime();
        sentCommand = jo;
        waitingCommandId = putDownId;
		putDownId.CreateStringWME("status", "sent");
        System.out.println("PUT DOWN: " + x + ", " + y + ", " + z);
    }

    private void processPointCommand(Identifier pointId)
    {
    	String objHandleStr = SoarUtil.getValueOfAttribute(pointId, "object-handle",
    			"Error (point): No ^object-handle attribute");
    	ArmPerceptionConnector perc = (ArmPerceptionConnector)soarAgent.getPerceptionConnector();
        Integer percId = perc.getWorld().getPerceptionId(Integer.parseInt(objHandleStr));
        if(percId == null){
        	System.err.println("Set: unknown handle " + objHandleStr);
        	pointId.CreateStringWME("status", "error");
        	return;
        }

        JsonObject jo = Json.createObjectBuilder()
            .add("utime", TimeUtil.utime())
            .add("action", String.format("POINT=%d", percId))
            .build();
        Message m = new Message(jo);
        armCommands.publish(m);

        sentTime = TimeUtil.utime();
        sentCommand = jo;
        waitingCommandId = pointId;
		pointId.CreateStringWME("status", "sent");
        System.out.println("POINT TO: " + percId +
                           " (Soar Handle: " + objHandleStr + ")");
    }

    private void processHomeCommand(Identifier id){
        JsonObject jo = Json.createObjectBuilder()
            .add("utime", TimeUtil.utime())
            .add("action", "HOME")
            .build();
        Message m = new Message(jo);
        armCommands.publish(m);
        id.CreateStringWME("status", "sent");
    }

    private void processResetCommand(Identifier id){
        JsonObject jo = Json.createObjectBuilder()
            .add("utime", TimeUtil.utime())
            .add("action", "RESET")
            .build();
        Message m = new Message(jo);
        armCommands.publish(m);
        id.CreateStringWME("status", "sent");
    }

    /**
     * Getters
     **/

	public Integer getHeldObject() {
		return heldObject;
	}

    public String getStatus(){
    	if(curStatus == null){
    		return "wait";
    	}
        return curStatus.getString("action").toLowerCase();
    }

    /**
     * Why the f is this a necessary override?
     **/
    @Override
    public void createMenu(JMenuBar menuBar){
    	JMenu actionMenu = new JMenu("Action");
    	JButton armResetButton  = new JButton("Reset Arm");
        armResetButton.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				//new ResetRobotArm().execute();
                System.out.println("Scripts are broken.");
			}
        });
        actionMenu.add(armResetButton);

        menuBar.add(actionMenu);
    }

	/*
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
        	id.CreateStringWME("status", "error");
        	return;
        }

        String name = SoarUtil.getValueOfAttribute(id,
                "name", "Error (set-state): No ^name attribute");
        String value = SoarUtil.getValueOfAttribute(id, "value",
                "Error (set-state): No ^value attribute");
		
        JsonObject jo = Json.createObjectBuilder()
            .add("utime", TimeUtil.utime())
            .add("objectId", percId)
			.add("stateName", name)
			.add("stateValue", value)
            .build();
        Message m = new Message(jo);
        simCommands.publish(m);
		setStateSentTime = TimeUtil.utime();
        id.CreateStringWME("status", "sent");
    }
}
