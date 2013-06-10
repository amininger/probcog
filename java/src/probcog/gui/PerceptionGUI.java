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
    PeriodicTasks tasks = new PeriodicTasks(2);   // Only one thread for now

    // GUI Stuff
    JMenuBar menuBar;
    JMenu controlMenu, editMenu;
    JMenuItem clearData, reloadData;
    JMenuItem undoAction, redoAction;

    public PerceptionGUI(GetOpt opts)
    {
        super("BOLT");
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);

        // Handle Options
        Config config;
        try {
            config = new ConfigFile(opts.getString("config"));
        } catch (IOException ioex) {
            System.err.println("ERR: Could not load configuration from file");
            ioex.printStackTrace();
            return;
        }
        if (opts.getString("backup") != null) {
            try {
                System.out.println("ATTN: Loading from autosave file");
                classifierManager.readState(opts.getString("backup"));
                System.out.println("ATTN: Successfully restored from autosave file");
            } catch (IOException ioex) {
                System.err.println("ERR: Failure to load from autosave file");
                ioex.printStackTrace();
            }
        }

        // Initialize object tracker
        try{
            tracker = new Tracker(config);
        }
        catch (IOException ioex)
        {
            System.err.println("ERR: Error config");
            ioex.printStackTrace();
        }
        // Initialize sensable manager
        //sensableManager = SensableManager.getSingleton();


        // If we specify that we would like to use an actual kinect,
        // initialize a kinect camera to process incoming kinect_status
        // messages. Otherwise, initialize a simulated kinect to generate
        // virtual point clouds based on the sim environment

    	// Initialize the simulator
        simulator = new ProbCogSimulator(opts, tracker);

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
        opts.addString('c', "config", null, "Specify the configuration file for Bolt");
        opts.addBoolean('d', "debug", false, "Toggle debugging mode");
        opts.addBoolean('k', "kinect", false, "Use kinect data to create objects");
        opts.addBoolean('a', "arm", false, "Run with the actual arm");
        opts.addBoolean('\0', "seg", false, "Show the segmentation instead of the simulator");
        opts.addString('w', "world", null, "World file");
        opts.addString('s', "sim-config", null, "Configuration file for the Simulator");
        opts.addString('\0', "backup", null, "Load from backup file");
        opts.addInt('\0', "fps", 10, "Maximum frame rate");

        if (!opts.parse(args) || opts.getBoolean("help") || opts.getExtraArgs().size() > 0) {
            opts.doHelp();
            return;
        }

        Config config;
        if (opts.getString("config") == null) {
            System.out.println("Usage: Must specify a configuration file");
            opts.doHelp();
            return;
        } else {
            try {
                config = new ConfigFile(opts.getString("config"));
            } catch (IOException ioex) {
                System.err.println("ERR: "+ioex);
                ioex.printStackTrace();
                return;
            }
        }

        // Initialize the arm
        // BoltArm.getSingleton().initArm(config); // XXX - How do we initialize the arm now?

        PerceptionGUI gui = new PerceptionGUI(opts);

        ArmCommandInterpreter interpreter = new ArmCommandInterpreter(opts.getBoolean("debug"));

        if (opts.getBoolean("arm")) {
            try {
                ArmController controller = new ArmController(config);
                ArmDriver armDriver = new ArmDriver(config);
                (new Thread(armDriver)).start();
                if (opts.getBoolean("debug")) {
                    ArmDemo demo = new ArmDemo(config, false);
                }
            }
            catch (IOException ioex) {
                System.err.println("ERR: Error reading arm config");
                ioex.printStackTrace();
                return;
            }
        } else {
            if (opts.getBoolean("debug")) {
                try {
                    ArmDemo demo = new ArmDemo(config, true);
                }
                catch (IOException ioex) {
                    System.err.println("ERR: Error reading arm config");
                    ioex.printStackTrace();
                    return;
                }
            }
        }

        // if (opts.getBoolean("debug")) {
        //     ClassifyDebugGUI clDebugger = new ClassifyDebugGUI();
        // }
    }
}