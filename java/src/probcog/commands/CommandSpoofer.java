package probcog.commands;

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

public class CommandSpoofer {

    /*
		public static control_law_t createControlLaw(String clName, String testName, Double value){
			control_law_t cl = new control_law_t();
			cl.id = 1;
			cl.name = clName;
			cl.num_params = 0;
			cl.param_names = new String[0];
			cl.param_values = new typed_value_t[0];

            if(clName.equals("turn"))
            {
                cl.num_params = 1;
                cl.param_names = new String[]{"direction"};
                cl.param_values = new typed_value_t[1];
                if(value < 0)
                    cl.param_values[0] = TypedValue.wrap("right");
                else
                    cl.param_values[0] = TypedValue.wrap("left");
            }

			condition_test_t ct = new condition_test_t();
			ct.name = testName;
			ct.num_params = 0;
			ct.param_names = new String[0];
			ct.param_values = new typed_value_t[0];
			ct.compare_type = condition_test_t.CMP_GTE;
			ct.compared_value = TypedValue.wrap(value);

            if(clName.equals("turn") && value<0)
            {
                ct.compare_type = condition_test_t.CMP_LTE;
            }

			cl.termination_condition = ct;
			return cl;
		}

	public static void main(String[] args){
		if(args.length < 3){
			System.err.println("Need 3 args");
		}
		String clName = args[0];
		String ctName = args[1];
		Double value = new Double(args[2]);

		control_law_t cl = createControlLaw(clName, ctName, value);

		LCM lcm = LCM.getSingleton();
		lcm.publish("SOAR_COMMAND", cl);
	}*/
}
