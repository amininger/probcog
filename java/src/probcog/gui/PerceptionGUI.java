package probcog.gui;

import java.io.*;
import javax.swing.*;
import java.util.*;
import java.util.Timer;
import java.awt.event.*;
import java.text.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.LinAlg;
import april.sim.*;
import april.util.*;

import probcog.arm.*;
import probcog.lcmtypes.*;
import probcog.perception.*;
import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;


public class PerceptionGUI extends JFrame implements LCMSubscriber
{
    private ArmController controller;
    private Tracker tracker;
    private ClassifierManager classifierManager;
    private KinectView kinectView;
    private ProbCogSimulator simulator;

    // objects for visualization
    // private ArmSimulator armSimulator;

    // LCM
    static LCM lcm = LCM.getSingleton();
    private Timer sendObservationTimer;
    private static final int OBSERVATION_RATE = 2; // # sent per second

    // Periodic tasks
    PeriodicTasks tasks = new PeriodicTasks(2);

    // GUI Stuff
    JMenuBar menuBar;
    JMenu controlMenu, editMenu;
    JMenuItem clearData, reloadData;
    JMenuItem undoAction, redoAction;

    public PerceptionGUI(GetOpt opts) throws IOException
    {
        super("ProbCog");
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Handle Options
        Config config = new ConfigFile(opts.getString("config"));
        if (opts.getString("backup") != null) {
            System.out.println("ATTN: Loading from autosave file");
            classifierManager.readState(opts.getString("backup"));
            System.out.println("ATTN: Successfully restored from autosave file");
        }


        // Arm control...eventually should integrate with simulated arm?
        // XXX SEE ArmDemo
        controller = new ArmController(config);

        // XXX Arm debugging
        if (opts.getBoolean("debug")) {
            ArmDemo demo = new ArmDemo(config, false);  // XXX NO SIM
        }

        // Initialize sensable manager
        //sensableManager = SensableManager.getSingleton();


        // If we specify that we would like to use an actual kinect,
        // initialize a kinect camera to process incoming kinect_status
        // messages. Otherwise, initialize a simulated kinect to generate
        // virtual point clouds based on the sim environment

    	// Initialize the simulator
        simulator = new ProbCogSimulator(opts);

        // Initialize object tracker
        tracker = new Tracker(config, opts.getBoolean("kinect"), simulator.getWorld());


        if(opts.getBoolean("kinect")){
        	//camera = new KinectCamera(simulator.getVisWorld()); // Lauren
        	kinectView = new KinectView(tracker);
            // XXX We'd like to remove this middleman to the arm
            //BoltArmCommandInterpreter interpreter = new BoltArmCommandInterpreter(getSegment(), opts.getBoolean("debug"));
        }

        // XXX -- Need Sim stuff here
        // else {
        //     // XXX -- Need Sim Kinect here
        // 	camera = new SimKinect(400, 300, simulator);
        // }
        // if (!opts.getBoolean("arm")) {
        // 	armSimulator = new ArmSimulator(simulator);
        // }

        // Initialize the JMenuBar
        createMenuBar();
        simulator.addToMenu(menuBar);
        this.setJMenuBar(menuBar);
    	this.add(simulator.getCanvas());

        // Subscribe to LCM
        lcm.subscribe("TRAINING_DATA", this);
        lcm.subscribe("ROBOT_COMMAND", this);

        // TODO: arm stuff here

        this.setVisible(true);
        class SendObservationTask extends TimerTask{
			public void run() {
        		sendMessage();
			}
        }
        sendObservationTimer = new Timer();
        sendObservationTimer.schedule(new SendObservationTask(), 1000, 1000/OBSERVATION_RATE);

        // Write to file task
        tasks.addFixedDelay(new AutoSaveTask(), 10.0);
        tasks.addFixedDelay(new MenuUpdateTask(), 0.2);
        tasks.setRunning(true);
    }


    public void createMenuBar()
    {
    	menuBar = new JMenuBar();
        controlMenu = new JMenu("Control");

        menuBar.add(controlMenu);

        // Remove all data (no built in info)
        clearData = new JMenuItem("Clear All Data");
        clearData.addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    System.out.println("CLEARED DATA");
                    tracker.clearClassificationData();
                }
            });
        controlMenu.add(clearData);

        // Remove all data (including training)
        reloadData = new JMenuItem("Reload Data");
        reloadData.addActionListener(new ActionListener(){
                @Override
                public void actionPerformed(ActionEvent e) {
                    tracker.reloadClassificationData();
                }
            });
        controlMenu.add(reloadData);

        // Edit menu XXX
        editMenu = new JMenu("Edit");
        menuBar.add(editMenu);

        // Undo & redo actions
        undoAction = new JMenuItem("Undo");
        undoAction.addActionListener(new ActionListener()
            {
                public void actionPerformed(ActionEvent e) {
                    tracker.undoClassification();
                }
            });
        editMenu.add(undoAction);
        redoAction = new JMenuItem("Redo");
        redoAction.addActionListener(new ActionListener()
            {
                public void actionPerformed(ActionEvent e) {
                    tracker.undoClassification();
                }
            });
        editMenu.add(redoAction);
    }


    @Override
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
       if(channel.equals("TRAINING_DATA")){
            try{
                training_data_t training = new training_data_t(ins);

                for(int i=0; i<training.num_labels; i++){
                    training_label_t tl = training.labels[i];
                    Obj objTrain;
                    synchronized(tracker.stateLock){
                        objTrain = tracker.getObject(tl.id);
                    }
                    if(objTrain != null){
                        FeatureCategory cat = Features.getFeatureCategory(tl.cat.cat);
                        ArrayList<Double> features = objTrain.getFeatures(cat);
                        if(features != null){
                            tracker.addTraining(cat, features, tl.label);
                        }
                    }
                }
            }catch (IOException e) {
                e.printStackTrace();
                return;
            }
        } else if(channel.equals("ROBOT_COMMAND")){
        	try{
        		robot_command_t command = new robot_command_t(ins);
        		//sensableManager.performAction(command.action); XXX -- Sensables?
        	}catch (IOException e) {
                e.printStackTrace();
                return;
            }
        }
    }


    public void sendMessage()
    {
        observations_t obs = new observations_t();
        obs.utime = TimeUtil.utime();
        synchronized(tracker.stateLock){
        	obs.click_id = simulator.getSelectedId();
        }
        // XXX -- More commented out sensables
        // obs.sensables = sensableManager.getSensableStrings();
        // obs.nsens = obs.sensables.length;
        obs.observations = tracker.getObjectData();
        obs.nobs = obs.observations.length;

        lcm.publish("OBSERVATIONS",obs);
    }

    /** AutoSave the classifier state */
    class AutoSaveTask implements PeriodicTasks.Task
    {
        String filename;

        public AutoSaveTask()
        {
            Date date = new Date(System.currentTimeMillis());
            SimpleDateFormat sdf = new SimpleDateFormat("yyyy_MM_dd-HH_mm_ss");
            filename = "/tmp/probcog_autosave_"+sdf.format(date);
        }

        public void run(double dt)
        {
            try {
                tracker.writeClassificationState(filename);
            } catch (IOException ioex)  {
                System.err.println("ERR: Could not save to autosave file");
                ioex.printStackTrace();
            }
        }
    }

    /** Update the status of menu entries to ensure only the
     *  appropriate ones are marked as active
     */
    class MenuUpdateTask implements PeriodicTasks.Task
    {
        public MenuUpdateTask()
        {
        }

        public void run(double dt)
        {
            if (tracker.canUndoClassification()) {
                undoAction.setEnabled(true);
            } else {
                undoAction.setEnabled(false);
            }

            if (tracker.canRedoClassification()) {
                redoAction.setEnabled(true);
            } else {
                redoAction.setEnabled(false);
            }
        }
    }

    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        // XXX Todo: clean up arguments
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Global configuration file");
        opts.addString('w', "world", null, "Simulated world file");
        opts.addBoolean('k', "kinect", false, "Use a physical kinect");
        opts.addBoolean('d', "debug", false, "Toggle debugging mode");
        opts.addBoolean('e', "emulate", false, "Run a soar emulator that sends lcm messages");
        opts.addString('\0', "backup", null, "Load from backup file");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }

        SoarEmulator soarEm;
        if (opts.getBoolean("emulate"))
            soarEm = new SoarEmulator();

        // Spin up the GUI
        try {
            PerceptionGUI gui = new PerceptionGUI(opts);
        } catch (IOException ioex) {
            System.err.println("ERR: Error starting GUI");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}
