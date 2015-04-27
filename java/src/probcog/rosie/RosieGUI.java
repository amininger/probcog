package probcog.rosie;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.WindowAdapter;
import java.awt.event.WindowEvent;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.FileWriter;
import java.io.IOException;
import java.io.PrintWriter;
import java.util.List;
import java.util.Properties;

import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JMenuBar;

import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import sml.Agent;
import sml.Agent.PrintEventInterface;
import sml.Agent.RunEventInterface;
import sml.Kernel;
import sml.smlPrintEventId;
import sml.smlRunEventId;
import probcog.lcmtypes.*;
import april.util.GetOpt;
import april.util.TimeUtil;

public class RosieGUI extends JFrame
{
	public static class RosieConfig{
		public String agentName;

		public String agentSource;
		public String smemSource;

		public boolean spawnDebugger;
		public int watchLevel;
		public int throttleMS;
		
		public Boolean writeLog;

		public RosieConfig(String configFile, boolean debug) throws IOException{

	        // Load the properties file
	        Properties props = new Properties();
			props.load(new FileReader(configFile));
	        
	        spawnDebugger = debug;
	        		
	        agentName = props.getProperty("agent-name", "SoarAgent");
			agentSource = props.getProperty("agent-source", null);
			smemSource = props.getProperty("smem-source", null);
	        
	        try{
	        	watchLevel = Integer.parseInt(props.getProperty("watch-level", "1"));
	        } catch (NumberFormatException e){
	        	watchLevel = 1;
	        }
	        
	        try{
	        	throttleMS = Integer.parseInt(props.getProperty("decision-throttle-ms", "0"));
	        } catch(NumberFormatException e){
	        	throttleMS = 0;
	        }
	        
	        writeLog = props.getProperty("enable-log", "false").equals("true");
		}
	}
	
	private RosieConfig config;
	private SoarAgent soarAgent;
	
	private JButton startStopButton;
	
    public RosieGUI(RosieConfig config)
    {     
		super("Rosie Chat");
    	this.config = config;
        
    	soarAgent = new SoarAgent(config);
    	
    	this.setSize(800, 450);
        addWindowListener(new WindowAdapter() {
        	public void windowClosing(WindowEvent w) {
        		soarAgent.kill();
        	}
     	});
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

    	setupMenu();
    	this.add(new CommandPanel(soarAgent));
    	
    	this.setVisible(true);
    }
    
    private void setupMenu(){
    	JMenuBar menuBar = new JMenuBar();
    	
    	startStopButton = new JButton("START");
        startStopButton.addActionListener(new ActionListener(){
			public void actionPerformed(ActionEvent arg0) {
				if(soarAgent.isRunning()){
					startStopButton.setText("START");
					soarAgent.stop();
				} else {
					startStopButton.setText("STOP");
					soarAgent.start();
				}
			}
        });
        menuBar.add(startStopButton);
    	
    	menuBar.add(new AgentMenu(soarAgent));

    	this.setJMenuBar(menuBar);
    }
    
    public static void main(String[] args) {
    	
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", "config/rosie.config", "Rosie configuration file");
        opts.addBoolean('d', "debug", true, "Show the soar debugger");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }
       
        RosieConfig config;
        try{
        	config = new RosieConfig(opts.getString("config"), opts.getBoolean("debug"));
        } catch (IOException e){
			e.printStackTrace();
			return;
        }
        
        new RosieGUI(config);
    }
}
