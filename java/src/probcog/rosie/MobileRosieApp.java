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
import april.util.*;
import sml.*;
import sml.Agent.PrintEventInterface;

public class MobileRosieApp implements IMessageListener, PrintEventInterface
{
    private final int PORT = 7679;
	private SoarAgent soarAgent;

	private MobilePerceptionConnector perception;
	private MobileActuationConnector actuation;
	private LanguageConnector language;
	
	private Properties props;

    private boolean broadcasting = false;
	
	private IMessagePasser messagePasser;

    private StringBuilder soarOutputBuffer = new StringBuilder();
    private long lastOutputSent = 0;
    private final int MIN_SEND_TIME = 100000;

    public MobileRosieApp(Properties props)
    {
    	this.props = props;
    	
    	messagePasser = new MessageServer(PORT);
    	messagePasser.addMessageListener(this);
    	
    	createSoarAgent(props);

      broadcasting = props.getProperty("broadcast-output", "false").equals("true");
      if(broadcasting){
        soarAgent.getAgent().RegisterForPrintEvent(smlPrintEventId.smlEVENT_PRINT, this, null);
      }

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
        switch(message.type){
            case AGENT_COMMAND:
                handleAgentCommand(message.message);
                break;
            case SOAR_COMMAND:
                String output = soarAgent.sendCommand(message.message);
                if(messagePasser != null && output.trim().length() > 0){
                    messagePasser.sendMessage(output, LanguageConnector.MessageType.SOAR_OUTPUT);
                }
                break;
        }
	}

    private void handleAgentCommand(String cmd){
        if(cmd.equals("run")){
			soarAgent.start();
        } else if(cmd.equals("stop")){
			soarAgent.stop();
        }
    }

	@Override
	public void printEventHandler(int eventID, Object data, Agent agent, String message) {
        if(messagePasser != null){
            long now = TimeUtil.utime();
            soarOutputBuffer.append(message);
            if(now - lastOutputSent > MIN_SEND_TIME){
                messagePasser.sendMessage(soarOutputBuffer.toString(), LanguageConnector.MessageType.SOAR_OUTPUT);
                soarOutputBuffer = new StringBuilder();
                lastOutputSent = now;
            }
        }
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
