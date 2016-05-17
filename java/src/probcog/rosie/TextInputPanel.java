package probcog.rosie;

import java.util.*;

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

import javax.swing.*;
import javax.swing.text.*;

import edu.umich.rosie.language.LanguageConnector.MessageType;

public abstract class TextInputPanel extends JPanel {
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

      // Returns the current string in the text field
      public String getCurrent(){
          return textField.getText().trim();
      }

      // Adds the current textfield string to the history,
      // Clears the textField, and set the index to the end
      public void advance(){
          String message = textField.getText().trim();
          if(index >= history.size() || !history.get(index).equals(message)){
              history.add(textField.getText());
          }
          index = history.size();
          textField.setText("");
      }

      // Sets the textfield to the next item in the history
      // (or empty if at the end)
      public void next(){
        if(index + 1 < history.size()){
          index++;
          textField.setText(history.get(index));
        } else {
          textField.setText("");
        }
      }

      // Sets the textfield to the previous item in the history
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
            onSendClicked();
        }
      }

      public void keyReleased(KeyEvent arg0) {
      }
    }; 

    public TextInputPanel(String title, int width, int height) {
      inputText = new JTextField();
      sendButton = new JButton("Send");
      textPane = new JTextPane();
      history = new ChatHistory(inputText);

      doc = (StyledDocument)textPane.getDocument();
      setupStyles();

      setupGUI(title, width, height);
    }

    private void onSendClicked(){
        String message = history.getCurrent();
        if(message.length() > 0){
            history.advance();
            sendMessage(message);
        }
    }

    public abstract void sendMessage(String message);

    public abstract void receiveMessage(String message, MessageType type);

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
            textPane.select(end, end);
        }
    }

    /**************************************************
     * Code for setting up the Panel and its GUI elements
     */
    
    private void setupGUI(String title, int width, int height){
        // size
        width = (width < 200 ? 200 : width);    // width at least 200
        height = (height < 150 ? 150 : height); // height at least 150
        this.setSize(width, height);

        // inputText
        inputText.setFont(new Font("Serif", Font.PLAIN, 18));
        inputText.setText("");
        inputText.requestFocus();

        // textPane
        textPane.setEditable(false);
        DefaultCaret caret = (DefaultCaret)textPane.getCaret();
        caret.setUpdatePolicy(DefaultCaret.ALWAYS_UPDATE);

        JScrollPane pane = new JScrollPane(textPane);
        pane.setViewportView(textPane);

        // sendButton
        sendButton.setBackground(new Color(150, 255, 150));
        sendButton.addActionListener(new ActionListener()
        {
            public void actionPerformed(ActionEvent e)
            {
                onSendClicked();
            }
        });

        // keyAdapter and listeners
        ChatKeyAdapter keyAdapter = new ChatKeyAdapter(history);
        textPane.addKeyListener(keyAdapter);
        inputText.addKeyListener(keyAdapter);
        sendButton.addKeyListener(keyAdapter);

        // layout and split panes
        JSplitPane pane2 = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,
                inputText, sendButton);
        JSplitPane pane1 = new JSplitPane(JSplitPane.VERTICAL_SPLIT, pane,
                pane2);

        pane1.setDividerLocation(height-100);
        pane2.setDividerLocation(width-100);
        
        this.setLayout(new BorderLayout());
        this.add(pane1, BorderLayout.CENTER);

        this.add(new JLabel(title), BorderLayout.PAGE_START);
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
        StyleConstants.setFontFamily(soarOutputStyle, "Monospaced");
    }    
}
