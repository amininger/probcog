package probcog.rosie;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import javax.swing.JButton;
import javax.swing.JMenu;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import probcog.arm.ArmStatus;
import probcog.commands.TypedValue;
import probcog.lcmtypes.robot_action_t;
import probcog.lcmtypes.robot_command_t;
import probcog.lcmtypes.set_state_command_t;
import probcog.lcmtypes.control_law_t;
import probcog.lcmtypes.control_law_status_t;
import probcog.lcmtypes.condition_test_t;
import probcog.lcmtypes.typed_value_t;
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

import probcog.rosie.world.Pose;
import probcog.rosie.world.SVSCommands;
import probcog.rosie.world.WMUtil;
import probcog.rosie.world.WorldModel;

public class MotorSystemConnector   implements OutputEventInterface, RunEventInterface, LCMSubscriber{
	private SoarAgent agent;
    private Identifier inputLinkId;
	private Identifier selfId;

	private Pose pose;

	private robot_action_t curStatus = null;
	private robot_action_t prevStatus = null;
	// Last received information about the arm
	//

	private HashMap<Integer, Identifier> outstandingCommands;
	private int nextCommandId = 1;

	private boolean gotUpdate = false;

    private LCM lcm;

    private ArmStatus armStatus;

    StringBuilder svsCommands = new StringBuilder();



    public MotorSystemConnector(SoarAgent agent){
    	this.agent = agent;
    	pose = new Pose();

			outstandingCommands = new HashMap<Integer, Identifier>();

    	if(agent.getArmConfig() == null){
    		armStatus = null;
    	} else {
	        try {
	  			Config config = new ConfigFile(agent.getArmConfig());
	  			armStatus = new ArmStatus(config);
	  		} catch (IOException e) {
	  			armStatus = null;
	  		}
    	}

    	// Setup LCM events
        lcm = LCM.getSingleton();
        lcm.subscribe("ROBOT_ACTION", this);
				lcm.subscribe("SOAR_COMMAND_STATUS", this);

        // Setup Input Link Events
        inputLinkId = agent.getAgent().GetInputLink();
        agent.getAgent().RegisterForRunEvent(smlRunEventId.smlEVENT_BEFORE_INPUT_PHASE, this, null);

        // Setup Output Link Events
        String[] outputHandlerStrings = { "pick-up", "put-down", "point", "set-state", "home", "do-control-law"};
        for (String outputHandlerString : outputHandlerStrings)
        {
        	agent.getAgent().AddOutputHandler(outputHandlerString, this, null);
        }
    }

    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
			try {
				if(channel.equals("ROBOT_ACTION")){
						robot_action_t action = new robot_action_t(ins);
					newRobotStatus(action);
				} else if(channel.equals("SOAR_COMMAND_STATUS")){

						control_law_status_t status = new control_law_status_t(ins);
						newControlLawStatus(status);
				}
			} catch (IOException e) {
				e.printStackTrace();
			}
    }

    public void newRobotStatus(robot_action_t status){
    	curStatus = status;
    	gotUpdate = true;
    }

		public void newControlLawStatus(control_law_status_t status){
			if(outstandingCommands.containsKey(status.id)){
				Identifier id = outstandingCommands.get(status.id);
				boolean remove = false;
				String newStatus;
				if(status.status.equals("started")){
					newStatus = "received";
				} else if(status.status.equals("complete")){
					newStatus = "complete";
					remove = true;
				} else if(status.status.equals("early-termination")){
					newStatus = "error";
					WMUtil.updateStringWME(id, "error-type", "early-termination");
					remove = true;
				} else if(status.status.equals("failure")){
					newStatus = "error";
					WMUtil.updateStringWME(id, "error-type", "failure");
					remove = true;
				} else if(status.status.equals("unknown-command")){
					newStatus = "error";
					WMUtil.updateStringWME(id, "error-type", "unknown-command");
					remove = true;
				} else {
					return;
				}
				WMUtil.updateStringWME(id, "status", newStatus);
				if(remove){
					outstandingCommands.remove(status.id);
				}
			}
		}


    public String getStatus(){
    	if(curStatus == null){
    		return "wait";
    	} else {
    		return curStatus.action.toLowerCase();
    	}
    }

	// Happens during an input phase
	public synchronized void runEventHandler(int eventID, Object data, Agent agent, int phase){
    	long time = 0;
    	if(Rosie.DEBUG_TRACE){
    		time = TimeUtil.utime();
    	}

		if(selfId == null){
			initIL();
		} else if(gotUpdate){
			updateIL();
			gotUpdate = false;
		}
		if(armStatus != null){
			updateArmInfo();
		}
		if(svsCommands.length() > 0){
			agent.SendSVSInput(svsCommands.toString());
			//System.out.println(svsCommands.toString());
			svsCommands = new StringBuilder();
		}
		this.agent.commitChanges();
    	if(Rosie.DEBUG_TRACE){
			System.out.println(String.format("%-20s : %d", "MOTOR CONNECTOR", (TimeUtil.utime() - time)/1000));
    	}
	}

    private void initIL(){
    	selfId = inputLinkId.CreateIdWME("self");
    	selfId.CreateStringWME("action", "wait");
    	selfId.CreateStringWME("prev-action", "wait");
    	selfId.CreateStringWME("holding-obj", "false");
    	selfId.CreateIntWME("grabbed-object", -1);
    	pose.updateWithArray(new double[]{0, 0, 0, 0, 0, 0});
    	pose.updateInputLink(selfId);

    	if(armStatus != null){
        	svsCommands.append("a arm object world p 0 0 0 r 0 0 0\n");

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

        		svsCommands.append("a " + name + " object arm p 0 0 0 r 0 0 0 ");
        		svsCommands.append("s " + size[0] + " " + size[1] + " " + size[2] + " ");
        		svsCommands.append("v " + SVSCommands.bboxVertices() + "\n");
        	}
    	}
    }



    private void updateIL(){
    	WMUtil.updateStringWME(selfId, "action", curStatus.action.toLowerCase());
    	if(prevStatus == null){
        	WMUtil.updateStringWME(selfId, "prev-action", "wait");
    	} else {
        	WMUtil.updateStringWME(selfId, "prev-action", prevStatus.action.toLowerCase());
    	}
    	WMUtil.updateStringWME(selfId, "holding-obj", (curStatus.obj_id != -1 ? "true" : "false"));
    	WMUtil.updateIntWME(selfId, "grabbed-object", curStatus.obj_id);
    	//TODO remove
    	//stub for getting robot position on input link to test mobile robot
    	WorldModel world;
    	if ((world = agent.getWorldModel()) != null)
    	{
    		WMUtil.updateFloatWME(selfId, "robot-x", world.getRobotPose()[0]);
    		WMUtil.updateFloatWME(selfId, "robot-y", world.getRobotPose()[1]);
    	}
    	pose.updateWithArray(curStatus.xyz);
    	pose.updateInputLink(selfId);
    	prevStatus = curStatus;
    }

    private void updateArmInfo(){
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
            if (wme.GetAttribute().equals("set-state")) {
                processSetCommand(id);
            }
            else if (wme.GetAttribute().equals("pick-up")) {
                processPickUpCommand(id);
            }
            else if (wme.GetAttribute().equals("put-down")) {
                processPutDownCommand(id);
            }
            else if (wme.GetAttribute().equals("point")) {
                processPointCommand(id);
            }
            else if(wme.GetAttribute().equals("home")){
            	processHomeCommand(id);
            }
						else if(wme.GetAttribute().equals("do-control-law")){
							processDoControlLawCommand(id);
						}
            agent.commitChanges();
        } catch (IllegalStateException e){
        	System.out.println(e.getMessage());
        }
	}

    /**
     * Takes a pick-up command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command. Expects pick-up
     * ^object-id [int]
     */
    private void processPickUpCommand(Identifier pickUpId)
    {
        String objectIdStr = WMUtil.getValueOfAttribute(pickUpId,
                "object-id", "pick-up does not have an ^object-id attribute");

        robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime();
        command.action = String.format("GRAB=%d", Integer.parseInt(objectIdStr));
        command.dest = new double[6];
    	lcm.publish("ROBOT_COMMAND", command);
        pickUpId.CreateStringWME("status", "complete");
    }

    /**
     * Takes a put-down command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command Expects put-down
     * ^location <loc> <loc> ^x [float] ^y [float] ^z [float]
     */
    private void processPutDownCommand(Identifier putDownId)
    {
        Identifier locationId = WMUtil.getIdentifierOfAttribute(
                putDownId, "location",
                "Error (put-down): No ^location identifier");
        double x = Double.parseDouble(WMUtil.getValueOfAttribute(
                locationId, "x", "Error (put-down): No ^location.x attribute"));
        double y = Double.parseDouble(WMUtil.getValueOfAttribute(
                locationId, "y", "Error (put-down): No ^location.y attribute"));
        double z = Double.parseDouble(WMUtil.getValueOfAttribute(
                locationId, "z", "Error (put-down): No ^location.z attribute"));
        robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime();
        command.action = "DROP";
        command.dest = new double[]{x, y, z, 0, 0, 0};
    	lcm.publish("ROBOT_COMMAND", command);
        putDownId.CreateStringWME("status", "complete");
    }

    /**
     * Takes a set-state command on the output link given as an identifier and
     * uses it to update the internal robot_command_t command
     */
    private void processSetCommand(Identifier id)
    {
        String objId = WMUtil.getValueOfAttribute(id, "id",
                "Error (set-state): No ^id attribute");
        String name = WMUtil.getValueOfAttribute(id,
                "name", "Error (set-state): No ^name attribute");
        String value = WMUtil.getValueOfAttribute(id, "value",
                "Error (set-state): No ^value attribute");

        String action = String.format("ID=%s,%s=%s", objId, name, value);
        set_state_command_t command = new set_state_command_t();
        command.utime = TimeUtil.utime();
        command.state_name = name;
        command.state_val = value;
        command.obj_id = Integer.parseInt(objId);
    	lcm.publish("SET_STATE_COMMAND", command);
        id.CreateStringWME("status", "complete");
    }

    private void processPointCommand(Identifier pointId)
    {
    	Integer id = Integer.parseInt(WMUtil.getValueOfAttribute(pointId, "id"));

        robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime();
        command.dest = new double[]{0, 0, 0, 0, 0, 0};
    	command.action = "POINT=" + id;
    	lcm.publish("ROBOT_COMMAND", command);
        pointId.CreateStringWME("status", "complete");
    }

    private void processHomeCommand(Identifier id){
    	robot_command_t command = new robot_command_t();
        command.utime = TimeUtil.utime();
        command.dest = new double[6];
    	command.action = "HOME";
    	lcm.publish("ROBOT_COMMAND", command);
        id.CreateStringWME("status", "complete");
    }

		private void processDoControlLawCommand(Identifier id){
			control_law_t cl = parseControlLaw(id);
			if(cl == null){
				System.err.println("Invalid control law");
				id.CreateStringWME("status", "error");
				id.CreateStringWME("error-type", "parsing");
				return;
			}
			System.out.println("ISSUING COMMAND: " + cl.id);

			lcm.publish("SOAR_COMMAND", cl);
			System.out.println("PUBLISHED");
			outstandingCommands.put(cl.id, id);
			id.CreateStringWME("status", "sent");
		}

		public control_law_t parseControlLaw(Identifier id){
			control_law_t cl = new control_law_t();

			// ID - only used internally for unique identification
			cl.id = nextCommandId++;

			// Name of the condition test
			cl.name = WMUtil.getValueOfAttribute(id, "name");
			if(cl.name == null){
				System.err.println("No ^name attribute on condition test");
				return null;
			}

			// Parameters of the control law
			HashMap<String, typed_value_t> params = parseParameters(id, "parameters");
			cl.num_params = params.size();
			cl.param_names = new String[cl.num_params];
			cl.param_values = new typed_value_t[cl.num_params];
			int i = 0;
			for(Map.Entry<String, typed_value_t> e : params.entrySet()){
				cl.param_names[i] = e.getKey();
				cl.param_values[i] = e.getValue();
				i++;
			}

			// Termination condition - when to stop
			Identifier termId = WMUtil.getIdentifierOfAttribute(id, "termination-condition");
			cl.termination_condition = parseConditionTest(termId);
			if(cl.termination_condition == null){
				System.err.println("Invalid termination condition");
				return null;
			}

			return cl;
		}

		public condition_test_t parseConditionTest(Identifier id){
			condition_test_t ct = new condition_test_t();

			// Null condition test
			if(id == null){
				ct.name = "null";
				ct.num_params = 0;
				ct.param_names = new String[0];
				ct.param_values = new typed_value_t[0];
				ct.compare_type = condition_test_t.CMP_EQ;
				ct.compared_value = TypedValue.wrap("");;
				return ct;
			}

			// Name of the condition test
			ct.name = WMUtil.getValueOfAttribute(id, "name");
			if(ct.name == null){
				System.err.println("No ^name attribute on condition test");
				return null;
			}

			// Parameters of the condition
			HashMap<String, typed_value_t> params = parseParameters(id, "parameters");
			ct.num_params = params.size();
			ct.param_names = new String[ct.num_params];
			ct.param_values = new typed_value_t[ct.num_params];
			int i = 0;
			for(Map.Entry<String, typed_value_t> e : params.entrySet()){
				ct.param_names[i] = e.getKey();
				ct.param_values[i] = e.getValue();
				i++;
			}

			// compare-type
			//   The type of comparison (gt, gte, eq, lte, lt)
			String compareType = WMUtil.getValueOfAttribute(id, "compare-type");
			if(compareType == null){
				System.err.println("No compare-type on condition test");
				return null;
			} else if(compareType.equals("gt")){
				ct.compare_type = condition_test_t.CMP_GT;
			} else if(compareType.equals("gte")){
				ct.compare_type = condition_test_t.CMP_GTE;
			} else if(compareType.equals("eq")){
				ct.compare_type = condition_test_t.CMP_EQ;
			} else if(compareType.equals("lte")){
				ct.compare_type = condition_test_t.CMP_LTE;
			} else if(compareType.equals("lt")){
				ct.compare_type = condition_test_t.CMP_LT;
			} else {
				System.err.println("Unknown compare-type on condition test");
			}

			// compared-value
			//   the value being compared against when evaluating the test
			String comparedValue = WMUtil.getValueOfAttribute(id, "compared-value");
			if(comparedValue == null){
				System.err.println("no compared-value on condition test");
				return null;
			}
			ct.compared_value = WMUtil.wrapTypedValue(comparedValue);

			return ct;
		}

		public HashMap<String, typed_value_t> parseParameters(Identifier id, String att){
			HashMap<String, typed_value_t> params = new HashMap<String, typed_value_t>();
			Identifier paramsId = WMUtil.getIdentifierOfAttribute(id, att);
			if(paramsId != null){
				for(int i = 0; i < paramsId.GetNumberChildren(); i++){
					WMElement wme = paramsId.GetChild(i);
					String name = wme.GetAttribute();
					String value = wme.GetValueAsString();
					params.put(name, WMUtil.wrapTypedValue(value));
				}
			}

			return params;

		}



    public JMenu createMenu(){
    	JMenu actionMenu = new JMenu("Action");
    	JButton armResetButton  = new JButton("Reset Arm");
        armResetButton.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
			// TODO:


			}
        });
        actionMenu.add(armResetButton);

        return actionMenu;
    }
}
