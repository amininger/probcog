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

import javax.swing.*;

import edu.umich.rosie.language.*;
import edu.umich.rosie.language.LanguageConnector.MessageType;
import edu.umich.rosie.language.Message.MessageClient;
import edu.umich.rosie.language.IMessagePasser.IMessageListener;
import edu.umich.rosie.language.IMessagePasser.RosieMessage;
import edu.umich.rosie.soar.SoarAgent;

public class RemoteTerminal extends JFrame implements IMessagePasser.IMessageListener{
    public final int PORT = 7679;

    private TextInputPanel chatInputPanel;
    private TextInputPanel soarInputPanel;
    
    // OTHER
    private Object outputLock = new Object();

    private MessageClient client;
    private String server;
    

    public RemoteTerminal(String server) {
        createChatInputPanel();
        createSoarInputPanel();
        createSoarButtons();

        JSplitPane splitPane = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT,
                chatInputPanel, soarInputPanel);

        splitPane.setDividerLocation(500);
        this.add(splitPane);

        this.setSize(1000, 600);
        this.setVisible(true);

        connectToServer(server, PORT);
    }

    private void createChatInputPanel(){
        chatInputPanel = new TextInputPanel("Chat Dialog", 500, 600){
            public void sendMessage(String message){
                addMessageToDocument("I: " + message + "\n", MessageType.INSTRUCTOR_MESSAGE);
                sendMessageToRosie(message, MessageType.INSTRUCTOR_MESSAGE);
            }

            public void receiveMessage(String message, MessageType type){
                String timestamp = (new SimpleDateFormat("mm:ss:SSS")).format(new Date());
                if(type == MessageType.AGENT_MESSAGE){
                    message = "R: " + message + "\n";
                    addMessageToDocument(message, type);
                }
            }
        };
    }

    private void createSoarInputPanel(){
        soarInputPanel = new TextInputPanel("Soar Terminal", 500, 600){
            public void sendMessage(String message){
                addMessageToDocument("\n> " + message + "\n", MessageType.SOAR_COMMAND);
                sendMessageToRosie(message, MessageType.SOAR_COMMAND);
            }

            public void receiveMessage(String message, MessageType type){
                if(type == MessageType.SOAR_OUTPUT){
                    addMessageToDocument(message, type);
                }
            }
        };
    }

    private void createSoarButtons(){
        JPanel buttonPanel = new JPanel();
        BoxLayout layout = new BoxLayout(buttonPanel, BoxLayout.LINE_AXIS);

        JButton runButton = new JButton("Run");
        runButton.addActionListener(new ActionListener(){
            public void actionPerformed(ActionEvent e){
                sendMessageToRosie("run", MessageType.AGENT_COMMAND);
            }
        });
        buttonPanel.add(runButton);

        JButton stopButton = new JButton("Stop");
        stopButton.addActionListener(new ActionListener(){
            public void actionPerformed(ActionEvent e){
                sendMessageToRosie("stop", MessageType.AGENT_COMMAND);
            }
        });
        buttonPanel.add(stopButton);

        JButton stepButton = new JButton("Step");
        stepButton.addActionListener(new ActionListener(){
            public void actionPerformed(ActionEvent e){
                sendMessageToRosie("step", MessageType.SOAR_COMMAND);
            }
        });
        buttonPanel.add(stepButton);

        JButton watchOffButton = new JButton("Watch 0");
        watchOffButton.addActionListener(new ActionListener(){
            public void actionPerformed(ActionEvent e){
                sendMessageToRosie("w 0", MessageType.SOAR_COMMAND);
            }
        });
        buttonPanel.add(watchOffButton);

        JButton watchOnButton = new JButton("Watch 1");
        watchOnButton.addActionListener(new ActionListener(){
            public void actionPerformed(ActionEvent e){
                sendMessageToRosie("w 1", MessageType.SOAR_COMMAND);
            }
        });
        buttonPanel.add(watchOnButton);

        soarInputPanel.add(buttonPanel, BorderLayout.PAGE_END);
    }

    private void connectToServer(String server, int port){
        System.out.println("RemoteTerminal: Connecting to " + server);
        try {
            client = new MessageClient();
            client.newConnection(Inet4Address.getByName(server), port);
            System.out.println("RemoteTerminal: Successfully connected");
        } catch (UnknownHostException e){
            System.out.println("RemoteTerminal: Tried connecting to unknown host");
            client = null;
        } catch (IOException e){
            System.out.println("RemoteTerminal: Could not establish a server connection");
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

    private void sendMessageToRosie(String message, MessageType msgType){
        if(client != null && client.isConnected()){
            client.sendMessage(message, msgType);
        }
    }
    
    @Override
    public void receiveMessage(RosieMessage message){
        switch(message.type){
            case AGENT_MESSAGE: 
                chatInputPanel.receiveMessage(message.message, message.type);
                break;
            case SOAR_OUTPUT: 
                soarInputPanel.receiveMessage(message.message, message.type);
                break;
        }
    }


    public static void main(String[] args){
      System.out.println(args.length);
      if(args.length < 1){
        new RemoteTerminal(null);
      } else {
        new RemoteTerminal(args[0]);
      }
    }
}
