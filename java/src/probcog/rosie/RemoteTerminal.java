package probcog.rosie;

import java.awt.BorderLayout;
import java.awt.Color;
import java.awt.Font;
import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;
import java.io.File;
import java.io.IOException;
import java.net.Inet4Address;
import java.net.UnknownHostException;
import java.text.DateFormat;
import java.text.SimpleDateFormat;
import java.util.ArrayList;
import java.util.Date;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;

import javax.sound.sampled.LineUnavailableException;
import javax.sound.sampled.AudioFileFormat;
import javax.sound.sampled.AudioFormat;
import javax.sound.sampled.AudioSystem;
import javax.sound.sampled.DataLine;
import javax.sound.sampled.TargetDataLine;
import javax.swing.JButton;
import javax.swing.JFrame;
import javax.swing.JScrollPane;
import javax.swing.JSplitPane;
import javax.swing.JTextField;
import javax.swing.JTextPane;
import javax.swing.text.BadLocationException;
import javax.swing.text.DefaultCaret;
import javax.swing.text.Style;
import javax.swing.text.StyleConstants;
import javax.swing.text.StyledDocument;
import javax.swing.JPanel;

import edu.umich.rosie.language.*;
import edu.umich.rosie.language.LanguageConnector.MessageType;
import edu.umich.rosie.language.Message.MessageClient;
import edu.umich.rosie.language.IMessagePasser.IMessageListener;
import edu.umich.rosie.language.IMessagePasser.RosieMessage;
import edu.umich.rosie.soar.SoarAgent;

public class RemoteTerminal extends JFrame implements IMessagePasser.IMessageListener{
  // GUI COMPONENTS
  
  private StyledDocument chatDoc;
    // The document which messages are displayed to

    private final JTextField soarText;
    // The text field the user can type messages to the agent in
    
    private final JTextPane soarPane;
    
    private final JButton sendButton;
    // The button used to send a message to the soar agent
    
    // OTHER
    private Object outputLock = new Object();

    private MessageClient client;
    

    /***********************************************
     * ChatHistory
     *   can manange the history of a set of messages
     ***********************************************/
    class ChatHistory{
      public final JTextField textField;
      public final JButton sendButton;
      public ArrayList<String> history;
      public int index;
      public ChatHistory(JTextField field, JButton button){
        textField = field;
        sendButton = button;
        history = new ArrayList<String>();
        index = 0;

        sendButton.addActionListener(new ActionListener()
        {
          public void actionPerformed(ActionEvent e)
          {
            String msg = textField.getText().trim();
            if(msg.length() == 0){
              return;
            }
            add(msg);
            sendMessageToRosie("CMD: " + msg);
          }
        });
      }

      public void add(String message){
        history.add(message);
        index = history.size();
        textField.setText("");
      }

      public void next(){
        if(index + 1 < history.size()){
          index++;
          textField.setText(history.get(index));
        } else {
          textField.setText("");
        }
      }

      public void prev(){
        if(index > 0){
          index--;
        }
        if(history.size() > index){
          textField.setText(history.get(index));
        }
      }
    }

    /*******************************************************
     * Handling Keyboard Events
     *   Up/Down - go up/down in history to repeat sentence
     */
    
    class ChatKeyAdapter extends KeyAdapter{
      private ChatHistory history;
      public ChatKeyAdapter(ChatHistory history){
        this.history = history;
      }

      public void keyPressed(KeyEvent arg0) {
        if(arg0.getKeyCode() == KeyEvent.VK_UP) {
          history.prev();
        } else if(arg0.getKeyCode() == KeyEvent.VK_DOWN){
          history.next();
        }
      }

      public void keyReleased(KeyEvent arg0) {
      }
    }; 

    public RemoteTerminal(String ip) {
      soarPane = new JTextPane();
      soarText = new JTextField();
      sendButton = new JButton("Send");

      setupGUI();
      setupStyles();
      createClient(ip, 7679);
    }

    private void createClient(String ip, int port){
      System.out.println("CREATING CLIENT: " + ip);
      try {
          client = new MessageClient();
          client.newConnection(Inet4Address.getByName(ip), port);
          System.out.println("Successfully Connected");
      } catch (UnknownHostException e){
          System.out.println("Tried connecting to unknown host");
          client = null;
      } catch (IOException e){
          System.out.println("Could not establish a server connection");
          client = null;
      }

      if (client != null){
        client.addMessageListener(this);
      }
    }
    
    /**********************************************************
     * Public Interface for interacting with the chat frame
     * 
     * registerNewMessage(String message, MessageSource src)
     *   Use to add a new message to the chat text field
     * clearMessages()
     *   Remove all messages from the text field
     */

    private void sendMessageToRosie(String message){
        if(client != null && client.isConnected()){
            client.sendMessage(message, LanguageConnector.MessageType.INSTRUCTOR_MESSAGE);
        }
    }
    
    @Override
    public void receiveMessage(RosieMessage message){
      synchronized(outputLock){
        Style msgStyle = chatDoc.getStyle(message.type.toString());
      DateFormat dateFormat = new SimpleDateFormat("mm:ss:SSS");
      Date d = new Date();
      
      String fullMessage = dateFormat.format(d) + " ";

      switch(message.type){
      case INSTRUCTOR_MESSAGE:
        fullMessage += "Mentor: ";
        break;
      case AGENT_MESSAGE:
        fullMessage += "Agent: " ;
        break;
      }

      if(message.type == MessageType.SOAR_OUTPUT){
        fullMessage = message.message;
      } else {
        fullMessage += message.message + "\n";
      }
      
      try{
        int origLength = chatDoc.getLength();
        chatDoc.insertString(origLength, fullMessage, msgStyle);
      } catch (BadLocationException e){
        // Should never encounter this
        System.err.println("Failed to add message to chat window");
      }
  
      // AM: Will make it auto scroll to bottom
      int end = chatDoc.getLength();
      soarPane.select(end, end);

        switch(message.type){
        case INSTRUCTOR_MESSAGE:
              soarText.setText("");
              soarText.requestFocus();
          break;
        case AGENT_MESSAGE:
          break;
        }
      }
    }

    
    /**************************************************
     * Code for setting up the Panel and its GUI elements
     */
    
    private void setupGUI(){
      soarPane.setEditable(false);

      soarText.setFont(new Font("Serif", Font.PLAIN, 18));

      DefaultCaret caret = (DefaultCaret)soarPane.getCaret();
      caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);

      JScrollPane pane = new JScrollPane(soarPane);
      pane.setViewportView(soarPane);

      // Send Button
      sendButton.setBackground(new Color(150, 255, 150));
      sendButton.setText("Send");

      ChatHistory soarHistory = new ChatHistory(soarText, sendButton);
      ChatKeyAdapter keyAdapter = new ChatKeyAdapter(soarHistory);
      soarPane.addKeyListener(keyAdapter);
      soarText.addKeyListener(keyAdapter);
      sendButton.addKeyListener(keyAdapter);
      this.getRootPane().setDefaultButton(sendButton);


      chatDoc = (StyledDocument)soarPane.getDocument();

      JSplitPane pane2 = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,
              soarText, sendButton);
      JSplitPane pane1 = new JSplitPane(JSplitPane.VERTICAL_SPLIT, pane,
              pane2);

      pane1.setDividerLocation(325);
      pane2.setDividerLocation(600);
      
      this.setLayout(new BorderLayout());
      this.add(pane1, BorderLayout.CENTER);
      this.setSize(800, 450);

      this.setVisible(true);

    }

    
    /*******************************
     * Setup Styles for how messages look
     */
    
    private void setupStyles() {
      // defaultStyle - Base style used by others
      Style defaultStyle = chatDoc.addStyle("DEFAULT", null);
        StyleConstants.setForeground(defaultStyle, Color.BLACK);
        StyleConstants.setFontSize(defaultStyle, 24);
        StyleConstants.setFontFamily(defaultStyle, "SansSerif");
        StyleConstants.setLineSpacing(defaultStyle, 1f);

      // agentStyle - Messages produced by the agent
      Style agentStyle = chatDoc.addStyle(MessageType.AGENT_MESSAGE.toString(), defaultStyle);
        StyleConstants.setForeground(agentStyle, Color.BLACK);
        StyleConstants.setItalic(agentStyle, true);
        StyleConstants.setFontFamily(agentStyle, "Serif");
      
      // instructorStyle - Messages typed by the user
        Style instructorStyle = chatDoc.addStyle(MessageType.INSTRUCTOR_MESSAGE.toString(), defaultStyle);
        StyleConstants.setForeground(instructorStyle, Color.BLACK);
    }    

    public static void main(String[] args){
      System.out.println(args.length);
      if(args.length < 1){
        new RemoteTerminal("127.0.0.1");
      } else {
        new RemoteTerminal(args[0]);
      }
    }
}
