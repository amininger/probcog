package probcog.rosie;

import java.awt.Color;
import java.awt.GridLayout;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.util.ArrayList;

import javax.swing.JButton;
import javax.swing.JLabel;
import javax.swing.JPanel;
import javax.swing.JTextField;

import sml.Agent;
import sml.Agent.RunEventInterface;
import sml.Identifier;
import sml.smlRunEventId;

public class CommandPanel extends JPanel implements RunEventInterface{
	
	private ArrayList<JButton> buttons;
	
	private JTextField objectText;
	private JTextField countText;
	
	Identifier commandLink;
	Identifier curCommand;
	
	boolean newCommand = false;
	String commandType = null;
	String objectType = null;
	Integer objectCount = null;
	String direction = null;
	

	public CommandPanel(SoarAgent agent){
        // Setup Input Link Events
        agent.getAgent().RegisterForRunEvent(smlRunEventId.smlEVENT_BEFORE_INPUT_PHASE, this, null);

		buttons = new ArrayList<JButton>();
		
		this.setLayout(new GridLayout(0, 3));

		/*
		 * Row 1: Follow-Left, Drive-Forward, Follow-Right
		*/
		JButton followLeft = new JButton("Follow Left Wall");
		followLeft.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendCommand("follow-left-wall1", true);
			}
		});
		followLeft.setBackground(new Color(150, 255, 150));
		this.add(followLeft);
		
		JButton driveForward = new JButton("Drive Forward");
		driveForward.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendCommand("drive-forward1", true);
			}
		});
		driveForward.setBackground(new Color(150, 255, 150));
		this.add(driveForward);

		JButton followRight = new JButton("Follow Right Wall");
		followRight.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendCommand("follow-right-wall1", true);
			}
		});	
		followRight.setBackground(new Color(150, 255, 150));
		this.add(followRight);
		
		/*
		 * Row 2: Turn-Left, Stop, Turn-Right
		*/
		JButton turnLeft = new JButton("Turn Left");
		turnLeft.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendCommand("turn-left1", false);
			}
		});
		turnLeft.setBackground(new Color(150, 150, 255));
		this.add(turnLeft);
		
		JButton stop = new JButton("Stop");
		stop.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendCommand("stop1", false);
			}
		});
		stop.setBackground(new Color(255, 150, 150));
		this.add(stop);
		
		JButton turnRight = new JButton("Turn Right");
		turnRight.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendCommand("turn-right1", false);
			}
		});
		turnRight.setBackground(new Color(150, 150, 255));
		this.add(turnRight);
		
		/*
		 * Row 3: Blank, Turn-Around, Compass
		*/
		this.add(new JPanel());
			
		JButton turnAround = new JButton("Turn Around");
		turnAround.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendCommand("turn-around1", false);
			}
		});
		turnAround.setBackground(new Color(150, 150, 255));
		this.add(turnAround);	
		
		this.add(setupCompassPanel());
		
		/*
		 * Row 4: Object Label, Blank, Count Label 
		*/
		this.add(new JLabel("Object to Count"));
		this.add(new JPanel());
		this.add(new JLabel("Count"));
		
		/*
		 * Row 4: Object TextField, Blank, Count TextField 
		*/
		objectText = new JTextField("door");
		this.add(objectText);
		
		this.add(new JPanel());
		
		countText = new JTextField("1");
		this.add(countText);
	}
	
	private JPanel setupCompassPanel(){
		JPanel compassPanel = new JPanel();

		compassPanel.setLayout(new GridLayout(0, 3));

		/*
		 * Row 1: Blank, North, Blank
		*/
		compassPanel.add(new JPanel());
			
		JButton north = new JButton("North");
		north.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendOrientCommand("north");
			}
		});
		north.setBackground(new Color(150, 150, 255));
		compassPanel.add(north);	
		
		compassPanel.add(new JPanel());

		/*
		 * Row 2: West, Text, East
		*/
		JButton west = new JButton("East");
		west.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendOrientCommand("west");
			}
		});
		west.setBackground(new Color(150, 150, 255));
		compassPanel.add(west);	
		
		compassPanel.add(new JLabel("Orient"));

		JButton east = new JButton("East");
		east.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendOrientCommand("east");
			}
		});
		east.setBackground(new Color(150, 150, 255));
		compassPanel.add(east);	

		/*
		 * Row 3: Blank, South, Blank
		*/
		compassPanel.add(new JPanel());
			
		JButton south = new JButton("South");
		south.addActionListener(new ActionListener(){
			@Override
			public void actionPerformed(ActionEvent arg0) {
				sendOrientCommand("south");
			}
		});
		south.setBackground(new Color(150, 150, 255));
		compassPanel.add(south);	
		
		compassPanel.add(new JPanel());
		
		return compassPanel;
	}
	
	public synchronized void sendCommand(String commandType, boolean includeObjectInfo){
		System.out.println("SENDING COMMAND: " + commandType);
		this.commandType = commandType;
		this.direction = null;
		if (includeObjectInfo){
			objectType = objectText.getText();
			try{
				objectCount = Integer.parseInt(countText.getText());
			} catch (NumberFormatException ex){
				return;
			}
		} else {
			objectType = null;
			objectCount = null;
		}
		newCommand = true;
	}
	
	public synchronized void sendOrientCommand(String direction){
		System.out.println("SENDING COMMAND: orient1");
		this.commandType = "orient1";
		this.objectType = null;
		this.objectCount = null;
		this.direction = direction;
		newCommand = true;
	}
	
	public synchronized void runEventHandler(int eventID, Object data, Agent agent, int phase){
		if (commandLink == null){
			commandLink = agent.GetInputLink().CreateIdWME("commands");
		}
		if (newCommand){
			if (curCommand != null){
				curCommand.DestroyWME();
			}
			curCommand = commandLink.CreateIdWME("command");
			curCommand.CreateStringWME("type", commandType);
			if (objectType != null){
				curCommand.CreateStringWME("object-type", objectType);
				curCommand.CreateIntWME("object-count", objectCount);
			}
			if (direction != null){
				curCommand.CreateStringWME("direction", direction);
			}
			agent.Commit();
			newCommand = false;
		}		
	}
}
