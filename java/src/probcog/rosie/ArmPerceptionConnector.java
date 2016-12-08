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

public class ArmPerceptionConnector extends AgentConnector {
	private static int SEND_TRAINING_FPS = 10;
	Timer sendTrainingTimer;

	// Object being pointed to
	private int pointedHandle = -1;
	private int currentTimer = -10;

    //x private HashMap<training_label_t, Identifier> outstandingTraining;

    //x private LCM lcm;

    protected WorldModel world;
    private String classifiersFile;

    public ArmPerceptionConnector(SoarAgent soarAgent, Properties props)
    {
    	super(soarAgent);

    	this.classifiersFile = props.getProperty("classifiers-file", "default");

    	//x outstandingTraining = new HashMap<training_label_t, Identifier>();

        world = new WorldModel(soarAgent);
        //x lcm = LCM.getSingleton();

        sendTrainingTimer = new Timer();

        String[] outputHandlerNames = new String[]{ "send-training-label", "modify-scene" };
        this.setOutputHandlerNames(outputHandlerNames);
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
        //x lcm.subscribe("OBSERVATIONS", this);
    }

    @Override
    public void disconnect(){
    	super.disconnect();

    	world.removeFromWM();
    	sendTrainingTimer.cancel();

        // ROSBRIDGE
        //x lcm.unsubscribe("OBSERVATIONS", this);
    }

    /*************************************************
     * handling training labels
     *************************************************/

    private void queueTrainingLabel(Integer objId, Integer cat, String label, Identifier id){
        return;
    	// training_label_t newLabel = new training_label_t();
    	// newLabel.utime = TimeUtil.utime();
    	// newLabel.id = objId;
    	// newLabel.cat = new category_t();
    	// newLabel.cat.cat = cat;
    	// newLabel.label = label;
    	//x synchronized(outstandingTraining){
    	// 	outstandingTraining.put(newLabel, id);
    	// }
    }

    private void sendTrainingLabels(){
        return;
    	//x synchronized(outstandingTraining){
    	// 	if(outstandingTraining.size() == 0){
    	// 		return;
    	// 	}
    	// 	training_data_t data = new training_data_t();
    	// 	data.utime = TimeUtil.utime();
    	// 	data.num_labels = outstandingTraining.size();
    	// 	data.labels = new training_label_t[data.num_labels];
    	// 	int i = 0;
    	// 	for(training_label_t label : outstandingTraining.keySet()){
    	// 		data.labels[i++] = label;
    	// 	}
        //     // ROSBRIDGE
    	// 	//x lcm.publish("TRAINING_DATA", data);
    	// }
    }

    private void receiveAckTime(long time){
        return;
    	// synchronized(outstandingTraining){
	    // 	Set<training_label_t> finishedLabels = new HashSet<training_label_t>();
	    // 	for(Map.Entry<training_label_t, Identifier> e : outstandingTraining.entrySet()){
	    // 		if(e.getKey().utime <= time){
	    // 			finishedLabels.add(e.getKey());
	    // 			e.getValue().CreateStringWME("status", "complete");
	    // 		}
	    // 	}
	    // 	for(training_label_t label : finishedLabels){
	    // 		outstandingTraining.remove(label);
	    // 	}
    	// }
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
        SoarUtil.updateIntWME(inputLink, "pointed-object", world.getSoarHandle(pointedHandle));
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
    	Integer catNum = PerceptualProperty.getPropertyID(propHandle);
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

    /*****************************************************
     * messageReceived
     * Gets a new observations_t message from LCM and sends it to the world
     ****************************************************/
    // x
    // public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    // {
    // 	if(channel.equals("OBSERVATIONS")){
    //         observations_t obs = null;
    //         try {
    //             obs = new observations_t(ins);
    //             pointedHandle = obs.click_id;
    //             world.newObservation(obs);
    //             receiveAckTime(obs.soar_utime);
    //         }
    //         catch (IOException e){
    //             e.printStackTrace();
    //             return;
    //         }
    // 	}
    // }

    public void createMenu(JMenuBar menuBar){
    	JMenu perceptionMenu = new JMenu("Perception");
    	JMenuItem clearDataButton = new JMenuItem("Clear Classifier Data");
        clearDataButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		// perception_command_t cmd = new perception_command_t();
        		// cmd.utime = TimeUtil.utime();
        		// cmd.command = "CLEAR_CLASSIFIERS";
                // LCM.getSingleton().publish("PERCEPTION_COMMAND", cmd);
                System.out.println("No LCM");
        	}
        });

        perceptionMenu.add(clearDataButton);

        JMenuItem loadDataButton = new JMenuItem("Load Classifier Data");
        loadDataButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		// perception_command_t cmd = new perception_command_t();
        		// cmd.utime = TimeUtil.utime();
        		// cmd.command = "LOAD_CLASSIFIERS=" + classifiersFile;
                // LCM.getSingleton().publish("PERCEPTION_COMMAND", cmd);
                System.out.println("No LCM");
        	}
        });

        perceptionMenu.add(loadDataButton);

        JMenuItem saveDataButton = new JMenuItem("Save Classifier Data");
        saveDataButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		// perception_command_t cmd = new perception_command_t();
        		// cmd.utime = TimeUtil.utime();
        		// cmd.command = "SAVE_CLASSIFIERS=" + classifiersFile;
                // LCM.getSingleton().publish("PERCEPTION_COMMAND", cmd);
                System.out.println("No LCM");
        	}
        });

        perceptionMenu.add(saveDataButton);

        JMenuItem loadDataFileButton = new JMenuItem("Load Data from File");
        loadDataFileButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		// String filename = JOptionPane.showInputDialog(null,
            	// 		  "Enter the filename to load from",
            	// 		  "Load Classifier Data From File",
            	// 		  JOptionPane.QUESTION_MESSAGE);
        		// perception_command_t cmd = new perception_command_t();
        		// cmd.utime = TimeUtil.utime();
        		// cmd.command = "LOAD_CLASSIFIERS=" + filename;
                // LCM.getSingleton().publish("PERCEPTION_COMMAND", cmd);
                System.out.println("No LCM");
        	}
        });

        perceptionMenu.add(loadDataFileButton);

        JMenuItem saveDataFileButton = new JMenuItem("Save Data from File");
        saveDataFileButton.addActionListener(new ActionListener(){
        	public void actionPerformed(ActionEvent e){
        		// String filename = JOptionPane.showInputDialog(null,
          		// 	  "Enter the filename to save to",
          		// 	  "Save Classifier Data To File",
          		// 	  JOptionPane.QUESTION_MESSAGE);
        		// perception_command_t cmd = new perception_command_t();
        		// cmd.utime = TimeUtil.utime();
        		// cmd.command = "SAVE_CLASSIFIERS=" + filename;
                // LCM.getSingleton().publish("PERCEPTION_COMMAND", cmd);
                System.out.println("No LCM");
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
