package probcog.rosie;

import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

import probcog.rosie.actuation.MobileActuationConnector;
import probcog.rosie.perception.MobilePerceptionConnector;
import edu.umich.rosie.language.IMessagePasser;
import edu.umich.rosie.language.IMessagePasser.IMessageListener;
import edu.umich.rosie.language.IMessagePasser.RosieMessage;
import edu.umich.rosie.language.LanguageConnector;
import edu.umich.rosie.language.Message.MessageServer;
import edu.umich.rosie.soar.SoarAgent;
import april.util.GetOpt;
import april.util.StringUtil;

public class MobileRosieApp implements IMessageListener
{
	private SoarAgent soarAgent;

	private MobilePerceptionConnector perception;
	private MobileActuationConnector actuation;
	private LanguageConnector language;
	
	private Properties props;
	
	private IMessagePasser messagePasser;

    public MobileRosieApp(Properties props)
    {
    	this.props = props;
    	
    	if(!props.getProperty("message-source", "lcm").equals("tablet")){
    		messagePasser = new LcmMessagePasser("robot");
    	} else {
    		messagePasser = new MessageServer("192.168.2.1", 7679);
    		messagePasser.addMessageListener(this);
    	}
    	
    	createSoarAgent(props);

    	while(true){
    		try{
    			Thread.sleep(1000);
    		} catch(InterruptedException e){}
    	}
    }
    
    private void createSoarAgent(Properties props){
    	soarAgent = new SoarAgent(props);
    	
    	actuation = new MobileActuationConnector(soarAgent, props);
    	soarAgent.setActuationConnector(actuation);
    	
    	perception = new MobilePerceptionConnector(soarAgent, props);
    	soarAgent.setPerceptionConnector(perception);
    	
    	language = new LanguageConnector(soarAgent, props, messagePasser);
    	soarAgent.setLanguageConnector(language);

    	soarAgent.createAgent();
    	soarAgent.start();
    }

	@Override
	public void receiveMessage(RosieMessage message) {
		System.out.println("GOT MESSAGE: " + message.message);
		if(message.message.startsWith("CMD: ")){
			String command = message.message.substring(5);
			if(command.equals("PAUSE") && soarAgent.isRunning()){
				soarAgent.stop();
			} else if (command.equals("RESUME") && !soarAgent.isRunning()){
				soarAgent.start();
			} else if(command.equals("RESTART")){
				soarAgent.kill();
				createSoarAgent(props);
			} else {
				System.out.println(soarAgent.sendCommand(command));
			}
		}
    	return;
	}

    public static void main(String[] args) {

        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addBoolean('d', "debug", true, "Show the soar debugger");
        opts.addString('c', "config", null, "The config file for Rosie");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }

        String configFile = opts.getString("config");
        if(configFile == null){
        	configFile = StringUtil.replaceEnvironmentVariables("$ROSIE_CONFIG");
        }
        System.out.println("CONFIG FILE = " + configFile);
        if(configFile.equals("")){
          System.err.println("ERR: No $ROSIE_CONFIG environment variable set");
          System.exit(1);
        }
        
        // Load the properties file
        Properties props = new Properties();
        try {
			props.load(new FileReader(configFile));
		} catch (IOException e) {
			System.out.println("File not found: " + configFile);
			e.printStackTrace();
			return;
		}

        new MobileRosieApp(props);
    }
}
