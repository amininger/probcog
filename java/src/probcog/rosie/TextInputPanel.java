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
import javax.swing.JPanel;
import javax.swing.JMenuBar;
import javax.swing.JMenu;
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

public class RemoteTerminal extends JPanel implements IMessagePasser.IMessageListener{
    protected final JTextField inputText;
    protected final JButton sendButton;
    protected final JTextPane textPane;
    protected final ChatHistory history;
    protected StyledDocument doc;

    /***********************************************
     * ChatHistory
     *   Manages the message history of a textField
     ***********************************************/
    class ChatHistory{
      private final JTextField textField;
      private ArrayList<String> history;
      private int index;
      public ChatHistory(JTextField field){
        textField = field;
        history = new ArrayList<String>();
        index = 0;
      }

      public String getCurrent(){
          if(index < history.size()){
              return history.get(index).trim();
          } else {
              return "";
          }
      }

      public void advance(){
          if(index < history.size()){
              history.add(history.get(index));
              index = history.size();
              textField.setText("");
          }
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
     *
     *******************************************************/
    
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
        } else if(arg0.getKeyCode() == KeyEvent.VK_ENTER){
            String message = history.getCurrent();
            if(message.length() > 0){
                sendMessage(message);
                history.advance();
            }
        }
      }

      public void keyReleased(KeyEvent arg0) {
      }
    }; 

    public TextInputPanel(String server) {
      inputText = new JTextField();
      sendButton = new JButton("Send");
      textField = new JTextPane();
      history = new ChatHistory(inputText);

      doc = (StyledDocument)textField.getDocument();
      setupStyles(doc);

      setupGUI();
    }

    private void 

    protected abstract void sendMessage(String message);

    public abstract void receiveMessage(RosieMessage message);

    protected void addMessageToDocument(String message, MessageType type){
        synchronized(doc){
            Style msgStyle = doc.getStyle(type.toString());
            if(msgStyle == null){
                return;
            }

            try{
                int origLength = doc.getLength();
                doc.insertString(origLength, message, msgStyle);
            } catch (BadLocationException e){
                // Should never encounter this
                System.err.println("Failed to add message to chat window");
            }

            // AM: Will make it auto scroll to bottom
            int end = doc.getLength();
            textField.select(end, end);
        }
    }

    /**************************************************
     * Code for setting up the Panel and its GUI elements
     */
    
    private void setupGUI(){
        textField.setEditable(false);
        inputText.setFont(new Font("Serif", Font.PLAIN, 18));

        DefaultCaret caret = (DefaultCaret)textField.getCaret();
        caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);

        JScrollPane pane = new JScrollPane(textField);
        pane.setViewportView(textField);

        sendSoarButton.setBackground(new Color(150, 255, 150));

        ChatKeyAdapter keyAdapter = new ChatKeyAdapter(soarHistory);
        textField.addKeyListener(keyAdapter);
        inputText.addKeyListener(keyAdapter);
        sendButton.addKeyListener(keyAdapter);
        this.getRootPane().setDefaultButton(sendButton);
              inputText.setText("");
              inputText.requestFocus();

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

        JSplitPane pane2 = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,
                inputText, sendButton);
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
      Style defaultStyle = doc.addStyle("DEFAULT", null);
        StyleConstants.setForeground(defaultStyle, Color.BLACK);
        StyleConstants.setFontSize(defaultStyle, 24);
        StyleConstants.setFontFamily(defaultStyle, "SansSerif");
        StyleConstants.setLineSpacing(defaultStyle, 1f);

      // agentStyle - Messages produced by the agent
      Style agentStyle = doc.addStyle(MessageType.AGENT_MESSAGE.toString(), defaultStyle);
        StyleConstants.setForeground(agentStyle, Color.BLACK);
        StyleConstants.setItalic(agentStyle, true);
        StyleConstants.setFontFamily(agentStyle, "Serif");
      
      // instructorStyle - Messages typed by the user
        Style instructorStyle = doc.addStyle(MessageType.INSTRUCTOR_MESSAGE.toString(), defaultStyle);
        StyleConstants.setForeground(instructorStyle, Color.BLACK);
      
      // agentCommand - Special commands sent to the Rosie program
        Style agentCommandStyle = doc.addStyle(MessageType.AGENT_COMMAND.toString(), defaultStyle);
        StyleConstants.setForeground(agentCommandStyle, Color.BLACK);
      
      // soarCommand - Messages typed by the user
        Style soarCommandStyle = doc.addStyle(MessageType.SOAR_COMMAND.toString(), defaultStyle);
        StyleConstants.setForeground(soarCommandStyle, Color.BLACK);

      // soarOutputStyle - output from soar
        Style soarOutputStyle = doc.addStyle(MessageType.SOAR_OUTPUT.toString(), defaultStyle);
        StyleConstants.setForeground(soarOutputStyle,  Color.BLACK);
        StyleConstants.setFontSize(soarOutputStyle, 16);
    }    
}
