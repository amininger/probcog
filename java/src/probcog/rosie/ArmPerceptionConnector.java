package probcog.rosie;

import java.awt.event.ActionEvent;
import java.awt.event.ActionListener;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Map;
import java.util.Properties;
import java.util.Set;
import java.util.Timer;
import java.util.TimerTask;

import javax.swing.JMenu;
import javax.swing.JMenuBar;
import javax.swing.JMenuItem;
import javax.swing.JOptionPane;

//x import lcm.lcm.LCM;
//x import lcm.lcm.LCMDataInputStream;
//x import lcm.lcm.LCMSubscriber;
import sml.Agent;
import sml.Agent.OutputEventInterface;
import sml.Agent.RunEventInterface;
import sml.Identifier;
import sml.WMElement;
import sml.smlRunEventId;
//x import probcog.lcmtypes.*;
import april.util.PeriodicTasks;
import april.util.TimeUtil;
import edu.umich.rosie.soar.*;
import probcog.rosie.perception.*;

import edu.wpi.rail.jrosbridge.*;
import edu.wpi.rail.jrosbridge.messages.*;
import edu.wpi.rail.jrosbridge.callback.*;
import javax.json.*;

public class ArmPerceptionConnector extends AgentConnector {
	private static int SEND_TRAINING_FPS = 10;
	Timer sendTrainingTimer;

	// Object being pointed to
	private int pointedHandle = -1;
	private int currentTimer = -10;

    private HashMap<TrainingLabel, Identifier> outstandingTraining;

    protected WorldModel world;
    private String classifiersFile;

    private Ros ros;
    private Topic perceptionComm;
    private Topic training;

    public ArmPerceptionConnector(SoarAgent soarAgent, Properties props)
    {
    	super(soarAgent);

    	this.classifiersFile = props.getProperty("classifiers-file", "default");

    	outstandingTraining = new HashMap<TrainingLabel, Identifier>();

        world = new WorldModel(soarAgent);

        sendTrainingTimer = new Timer();

        String[] outputHandlerNames = new String[]{ "send-training-label", "modify-scene" };
        this.setOutputHandlerNames(outputHandlerNames);

        ros = new Ros();
        ros.connect();

        if (ros.isConnected()) {
            System.out.println("ArmPerceptionConnector connected to rosbridge server.");
        }
        else {
            System.out.println("ArmPerceptionConnector NOT CONNECTED TO ROSBRIDGE");
        }
        perceptionComm = new Topic(ros,
                                   "/rosie_perception_commands",
                                   "std_msgs/String",
                                   500);
        training = new Topic(ros,
                             "/rosie_training",
                             "rosie_msgs/TrainingData",
                             500);
      }

    public WorldModel getWorld(){
    	return world;
    }

    @Override
    public void connect(){
        super.connect();

        sendTrainingTimer.schedule(new TimerTask(){
        	public void run(){
        		sendTrainingLabels();
        	}
        }, 1000, 1000/SEND_TRAINING_FPS);

        // ROSBRIDGE
        Topic observe = new Topic(ros,
                                 "/rosie_observations",
                                  "rosie_msgs/Observations");
        System.out.println("Subscribing to observations!");
        observe.subscribe(new TopicCallback() {
                public void handleMessage(Message message) {
                    JsonObject jobj = message.toJsonObject();
                    pointedHandle = jobj.getInt("click_id");
                    receiveAckTime(jobj.getInt("soar_utime"));
                    world.newObservation(jobj);
                }
            });
    }

    @Override
    public void disconnect(){
    	super.disconnect();

    	world.removeFromWM();
    	sendTrainingTimer.cancel();
    }

    /*************************************************
     * handling training labels
     *************************************************/

    private void queueTrainingLabel(Integer objId,
                                    CategorizedData.CategoryType cat,
                                    String label, Identifier id){
    	TrainingLabel newLabel = new TrainingLabel(objId, label, cat);
    	synchronized(outstandingTraining){
    		outstandingTraining.put(newLabel, id);
    	}
    }

    private void sendTrainingLabels(){
    	synchronized(outstandingTraining){
    		if(outstandingTraining.size() == 0){
    			return;
    		}
            StringBuilder outgoingLabels = new StringBuilder();
            outgoingLabels.append("{");

            long utime = TimeUtil.utime();
            outgoingLabels.append("\"header\": {\"stamp\": " + utime + "}, ");
            outgoingLabels.append("\"labels\": [");

            int i = 0;
    		for(TrainingLabel label : outstandingTraining.keySet()){
                if (i > 0) outgoingLabels.append(",");
                i++;
                outgoingLabels.append(label.toJsonString());
    		}
            outgoingLabels.append("]}");

            // System.out.println(outgoingLabels.toString());
            Message m = new Message(outgoingLabels.toString());
            training.publish(m);
    	}
    }

    private void receiveAckTime(long time){
    	synchronized(outstandingTraining){
	    	Set<TrainingLabel> finishedLabels = new HashSet<TrainingLabel>();
	    	for(Map.Entry<TrainingLabel, Identifier> e : outstandingTraining.entrySet()){
	    		if(e.getKey().getTime() <= time){
	    			finishedLabels.add(e.getKey());
	    			e.getValue().CreateStringWME("status", "complete");
	    		}
	    	}
	    	for(TrainingLabel label : finishedLabels){
	    		outstandingTraining.remove(label);
	    	}
    	}
    }

    /*************************************************
     * runEventHandler
     * Runs every input phase to update the input link
     * with perceptual information (time, pointed obj) as well as
     * Send training labels to perception
     *************************************************/
    long time = 0;

	@Override
    public synchronized void onInputPhase(Identifier inputLink){
        if(world.isAdded()){
        	world.updateWM();
        } else {
        	world.addToWM(inputLink);
        }

        // Update pointed object
        SoarUtil.updateStringWME(inputLink, "pointed-object", String.valueOf(world.getSoarHandle(pointedHandle)));
    }

    /*************************************************
     * outputEventHandlers
     *************************************************/

	@Override
	protected synchronized void onOutputEvent(String attName, Identifier id) {
		if(attName.equals("send-training-label")){
			processSendTrainingLabelCommand(id);
		} else if(attName.equals("modify-scene")){
			processModifySceneCommand(id);
		}
	}

    private void processSendTrainingLabelCommand(Identifier id){
    	Integer objHandle = Integer.parseInt(SoarUtil.getValueOfAttribute(id, "object-handle",
    			"Error (send-training-label): No ^object-handle attribute"));
    	String label = SoarUtil.getValueOfAttribute(id, "label",
    			"Error (send-training-label): No ^label attribute");
    	String propHandle = SoarUtil.getValueOfAttribute(id, "property-handle",
    			"Error (send-training-label): No ^property-handle attribute");

    	CategorizedData.CategoryType catNum = PerceptualProperty.getPropertyID(propHandle);
    	if(catNum == null){
    		id.CreateStringWME("status", "error");
    		System.err.println("ArmPerceptionConnector::processSendTrainingLabelCommand - bad category");
    		return;
    	}

    	queueTrainingLabel(objHandle, catNum, label, id);
    }


    private void processModifySceneCommand(Identifier rootId){
    	String type = SoarUtil.getValueOfAttribute(rootId, "type", "Error: No ^type attribute");
    	if(type.equals("link")){
    		Set<String> sourceHandles = SoarUtil.getAllValuesOfAttribute(rootId, "source-handle");
    		if(sourceHandles.size() == 0){
    			rootId.CreateStringWME("status", "error");
    			System.err.println("Error (link): No ^source-handle attribute");
    		}
    		String destHandle = SoarUtil.getValueOfAttribute(rootId, "destination-handle", 
    				"Error (link): No ^destination-handle attribute");
    		world.linkObjects(sourceHandles, destHandle);
    	} else {
    		rootId.CreateStringWME("status", "error");
    		System.err.println("ArmPerceptionConnector::processModifySceneCommand - bad type");
    		return;
    	}
    	rootId.CreateStringWME("status", "complete");
    }

    public void createMenu(JMenuBar menuBar){
    	JMenu perceptionMenu = new JMenu("Perception");
    	JMenuItem clearDataButton = new JMenuItem("Clear Classifier Data");
        clearDataButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		String commandmsg = "{\"data\": \"CLEAR_CLASSIFIERS\"}";
                Message m = new Message(commandmsg);
                perceptionComm.publish(m);
            }
        });

        perceptionMenu.add(clearDataButton);

        JMenuItem loadDataButton = new JMenuItem("Load Classifier Data");
        loadDataButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		String commandmsg = "{\"data\": \"LOAD_CLASSIFIERS=" +
                    classifiersFile + "\"}";
                Message m = new Message(commandmsg);
                perceptionComm.publish(m);
        	}
        });

        perceptionMenu.add(loadDataButton);

        JMenuItem saveDataButton = new JMenuItem("Save Classifier Data");
        saveDataButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		String commandmsg = "{\"data\": \"SAVE_CLASSIFIERS=" +
                    classifiersFile + "\"}";
                Message m = new Message(commandmsg);
                perceptionComm.publish(m);
        	}
        });

        perceptionMenu.add(saveDataButton);

        JMenuItem loadDataFileButton = new JMenuItem("Load Data from File");
        loadDataFileButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		String filename = JOptionPane.showInputDialog(null,
            			  "Enter the filename to load from",
            			  "Load Classifier Data From File",
            			  JOptionPane.QUESTION_MESSAGE);
        		String commandmsg = "{\"data\": \"LOAD_CLASSIFIERS=" +
                    filename + "\"}";
                Message m = new Message(commandmsg);
                perceptionComm.publish(m);
        	}
        });

        perceptionMenu.add(loadDataFileButton);

        JMenuItem saveDataFileButton = new JMenuItem("Save Data to File");
        saveDataFileButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		String filename = JOptionPane.showInputDialog(null,
          			  "Enter the filename to save to",
          			  "Save Classifier Data To File",
          			  JOptionPane.QUESTION_MESSAGE);
        		String commandmsg = "{\"data\": \"SAVE_CLASSIFIERS=" +
                    filename + "\"}";
                Message m = new Message(commandmsg);
                perceptionComm.publish(m);
        	}
        });

        perceptionMenu.add(saveDataFileButton);

        menuBar.add(perceptionMenu);
    }

	@Override
	protected void onInitSoar() {
		world.removeFromWM();
		//outstandingTraining.clear();
	}
}
