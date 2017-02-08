package probcog.rosie;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;

import javax.swing.JButton;
import javax.swing.JMenu;
import javax.swing.JOptionPane;

import april.util.TimeUtil;

public class EnvironmentMenu {
	public static JMenu createMenu() {
		JMenu environmentMenu = new JMenu("Environment");

		JButton worldResetButton = new JButton("Reset World");
		worldResetButton.addActionListener(new ActionListener() {
			public void actionPerformed(ActionEvent arg0) {
                System.out.println("Sorry, if you want a reset world action, you'll have to reimplement it.");
			}
		});
		environmentMenu.add(worldResetButton);

		return environmentMenu;
	}
}
