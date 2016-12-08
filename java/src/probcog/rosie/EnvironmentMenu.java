package probcog.rosie;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JMenu;
import javax.swing.JOptionPane;

//x import lcm.lcm.LCM;
//x import probcog.lcmtypes.perception_command_t;
//x import probcog.lcmtypes.robot_command_t;
//x import probcog.rosie.scripting.ResetEnvironmentState;
import april.util.TimeUtil;

public class EnvironmentMenu {
	public static JMenu createMenu() {
		JMenu environmentMenu = new JMenu("Environment");

		JButton worldResetButton = new JButton("Reset World");
		worldResetButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
                System.out.println("Removing LCM broke this");
				// robot_command_t command = new robot_command_t();
				// command.utime = TimeUtil.utime();
				// command.dest = new double[6];
				// command.action = "RESET";
				// LCM.getSingleton().publish("ROBOT_COMMAND", command);

				// perception_command_t pcmd = new perception_command_t();
				// pcmd.utime = TimeUtil.utime();
				// pcmd.command = "reset=time";
				// LCM.getSingleton().publish("GUI_COMMAND", pcmd);
			}
		});
		environmentMenu.add(worldResetButton);

		return environmentMenu;
	}
}
