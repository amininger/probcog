package probcog.rosie;

import java.util.ArrayList;
import java.util.Scanner;

import edu.umich.rosie.language.IMessagePasser;
import edu.umich.rosie.language.IMessagePasser.IMessageListener;
import edu.umich.rosie.language.IMessagePasser.RosieMessage;
import edu.umich.rosie.language.LanguageConnector.MessageType;
import edu.umich.rosie.language.LanguageConnector;

public class TerminalChat implements IMessageListener {
	IMessagePasser messagePasser;
	
	private ArrayList<String> messages;
	
	public TerminalChat(){
		messagePasser = new LcmMessagePasser("terminal");
		messagePasser.addMessageListener(this);
		
		messages = new ArrayList<String>();
		messages.add("Deliver the box to the soar office");
		messages.add("The goal is that the box is in the soar office");
		messages.add("Pick up the box");
		messages.add("Follow the right wall until you see two intersections");
		messages.add("Turn right");
		messages.add("Follow the right wall until you see an intersection");
		messages.add("You are at the soar office");
		messages.add("Go to the main office");
		messages.add("Deliver the kinect to the april office");
		messages.add("Go to the soar office");
		messages.add("Face north");
		messages.add("Follow the right wall until you see a door");
		messages.add("You are at the april office");
	}
	
	private void getTerminalInput(){
		Scanner scanner = new Scanner(System.in);
		while(true){
			for(int i = 0; i < messages.size(); i++){
				System.out.println(i + ": " + messages.get(i));
			}
			String nextLine = scanner.nextLine();
			try{
				Integer num = Integer.parseInt(nextLine);
				messagePasser.sendMessage(messages.get(num), MessageType.INSTRUCTOR_MESSAGE);
			} catch(NumberFormatException e){
				messagePasser.sendMessage(nextLine, MessageType.INSTRUCTOR_MESSAGE);
			}
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
