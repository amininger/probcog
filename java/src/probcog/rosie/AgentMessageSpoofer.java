package probcog.rosie;

import edu.umich.rosie.language.IMessagePasser.IMessageListener;
import edu.umich.rosie.language.IMessagePasser.RosieMessage;
import edu.umich.rosie.language.LanguageConnector.MessageType;
import edu.umich.rosie.language.Message.MessageServer;

public class AgentMessageSpoofer implements IMessageListener{
	
	MessageServer server;
	public AgentMessageSpoofer(){
		server = new MessageServer();
		server.addMessageListener(this);
	}

	@Override
	public void receiveMessage(RosieMessage message) {
		spoof();
	}
	
	
	public void spoof(){
		String[] messages = new String[]{
				"What is the goal?",
				"What do I do next?",
				"Please give me the box",
				"How do I get to the soar office?",
				"Ok",
				"Please take the box",
				"Please test me or give me another task",
				"Please give me the kinect",
				"How do I get to the april office?",
				"Please take the kinect",
				"Please give me the papers",
				"Please take the papers"
		};
		for (String msg : messages){
			server.sendMessage(msg, MessageType.AGENT_MESSAGE);
			try{
				Thread.sleep(3000);
			} catch(InterruptedException e){ }
		}
	}
	
	public static void main(String[] args){
		AgentMessageSpoofer spoofer = new AgentMessageSpoofer();
    	while(true){
    		try{
    			Thread.sleep(1000);
    		} catch(InterruptedException e){}
    	}
	}

}
