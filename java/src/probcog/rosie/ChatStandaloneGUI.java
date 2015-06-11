package probcog.rosie;

import javax.swing.JFrame;
import edu.umich.rosie.language.ChatPanel;

public class ChatStandaloneGUI extends JFrame
{
    public ChatStandaloneGUI()
    {
		super("Rosie Chat");
		
    	this.setSize(800, 400);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

    	ChatPanel chat = new ChatPanel(null, this);
    	this.add(chat);

    	//CommandPanel commandPanel = new CommandPanel(soarAgent);
    	
    	//JSplitPane splitPane = new JSplitPane(JSplitPane.VERTICAL_SPLIT, chat, commandPanel);
    	//splitPane.setDividerLocation(400);
    	//this.add(splitPane);
    	
    	this.setVisible(true);
    }

    public static void main(String[] args) {
        new ChatStandaloneGUI();
    }
}
