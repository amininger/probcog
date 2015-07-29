package probcog.rosie;

import java.util.Scanner;

import edu.umich.rosie.language.IMessagePasser;
import edu.umich.rosie.language.IMessagePasser.IMessageListener;
import edu.umich.rosie.language.IMessagePasser.RosieMessage;
import edu.umich.rosie.language.LanguageConnector.MessageType;
import edu.umich.rosie.language.LanguageConnector;

public class TerminalChat implements IMessageListener {
	IMessagePasser messagePasser;
	
	public TerminalChat(){
		messagePasser = new LcmMessagePasser("terminal");
		messagePasser.addMessageListener(this);
	}
	
	private void getTerminalInput(){
		Scanner scanner = new Scanner(System.in);
		while(true){
			String nextLine = scanner.nextLine();
			messagePasser.sendMessage(nextLine, MessageType.INSTRUCTOR_MESSAGE);
		}
	}
	
	@Override
	public void receiveMessage(RosieMessage message) {
		if(message.type == MessageType.AGENT_MESSAGE){
			System.out.println(message.message);
		}
	}
	
	public static void main(String[] args){
		TerminalChat chat = new TerminalChat();
		chat.getTerminalInput();
	}


}
