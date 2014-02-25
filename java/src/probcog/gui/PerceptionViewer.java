package probcog.gui;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferInt;
import java.io.*;
import javax.swing.*;
import java.text.*;
import java.util.*;
import java.util.Timer;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.util.*;
import april.vis.*;
import april.vis.VisCameraManager.CameraPosition;

import probcog.arm.*;
import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.*;
import probcog.perception.*;
import probcog.sensor.*;
import probcog.sim.SimLocation;
import probcog.sim.SimObjectPC;
import probcog.util.*;
import probcog.vis.*;

// import abolt.collision.ShapeToVisObject;
// import abolt.sim.SimSensable;
// import abolt.util.SimUtil;
// import abolt.objects.SensableManager;

public class PerceptionViewer extends JFrame implements LCMSubscriber
{
    private ProbCogSimulator simulator = null;
    private ArmStatus arm = null;

    // LCM
    static LCM lcm = LCM.getSingleton();

    // Vis Stuff
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    long kinectTime = 0;
    long updateKinectRate = 5;
    KinectView kinectView = null;

    // GUI Stuff
    JMenuBar menuBar;
    JMenu controlMenu, editMenu;

    Object stateLock = new Object();

    ArrayList<object_data_t> beliefObjects = new ArrayList<object_data_t>();
    ArrayList<object_data_t> perceptionObjects = new ArrayList<object_data_t>();

    Boolean drawPerceptionObjects;
    Boolean drawBeliefObjects;
    Boolean drawPropertyLabels;

    public PerceptionViewer(GetOpt opts) throws IOException
    {
        super("ProbCog Viewer");
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setLayout(new BorderLayout());

        // Handle Options
        Config config = new ConfigFile(opts.getString("config"));
        drawPerceptionObjects = opts.getBoolean("perception");
        drawBeliefObjects = opts.getBoolean("belief");
        drawPropertyLabels = opts.getBoolean("labels");

        // Arm control and arm monitor for rendering purposes

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        this.add(vc, BorderLayout.CENTER);

        if(opts.getBoolean("kinect")){
            kinectView = new KinectView(config, vw, vl);
        } else {
            simulator = new ProbCogSimulator(opts, vw, vl, vc);
            arm = new ArmStatus(config);
            new SimArm(config, simulator.getWorld());
        }

        // Initialize the JMenuBar
        createMenuBar();
        this.setJMenuBar(menuBar);

        // Subscribe to LCM
        lcm.subscribe("OBSERVATIONS", this);
        lcm.subscribe("SOAR_OBJECTS", this);

        this.setVisible(true);

        // Render updates about the world
        RenderThread rt = new RenderThread();
        rt.start();
    }


    public void createMenuBar()
    {
    	menuBar = new JMenuBar();

    	JMenu menu = new JMenu("View Options");
    	menu.add(new JLabel("Change Simulator View"));

    	JCheckBox percObjs = new JCheckBox("Perception Objects");
    	percObjs.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                if(e.getStateChange() == ItemEvent.DESELECTED) {
                	drawPerceptionObjects = false;
                	vw.getBuffer("perc-objects").clear();
                	vw.getBuffer("perc-labels").clear();
                }
                else if(e.getStateChange() == ItemEvent.SELECTED) {
                	drawPerceptionObjects = true;
                }
            }
        });
    	percObjs.setSelected(drawPerceptionObjects);
    	menu.add(percObjs);

    	JCheckBox propLabels = new JCheckBox("Property Labels");
    	propLabels.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                if(e.getStateChange() == ItemEvent.DESELECTED) {
                	drawPropertyLabels = false;
                	vw.getBuffer("perc-labels").clear();
                }
                else if(e.getStateChange() == ItemEvent.SELECTED) {
                	drawPropertyLabels = true;
                }
            }
        });
    	propLabels.setSelected(drawPropertyLabels);
    	menu.add(propLabels);

    	JCheckBox beliefObjs = new JCheckBox("Belief Objects");
    	beliefObjs.addItemListener(new ItemListener() {
            public void itemStateChanged(ItemEvent e) {
                if(e.getStateChange() == ItemEvent.DESELECTED) {
                	drawBeliefObjects = false;
                	vw.getBuffer("belief-objects").clear();
                	vw.getBuffer("belief-labels").clear();
                }
                else if(e.getStateChange() == ItemEvent.SELECTED) {
                	drawBeliefObjects = true;
                }
            }
        });
    	beliefObjs.setSelected(drawBeliefObjects);
    	menu.add(beliefObjs);

    	menuBar.add(menu);
    }

long time = 0;
    @Override
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
    	try{
	    	if(channel.equals("OBSERVATIONS")){
	    		observations_t percObjects = new observations_t(ins);
	    		handlePerceptionUpdate(percObjects);
	    	} else if(channel.equals("SOAR_OBJECTS")){
	    		soar_objects_t soarObjects = new soar_objects_t(ins);
	    		handleBeliefUpdate(soarObjects);
	    	} else if(channel.equals("KINECT_STATUS")){
	    		if(TimeUtil.utime() > kinectTime){
	    			kinectView.updateKinectInfo();
	    			kinectTime = TimeUtil.utime() + 1000000/updateKinectRate;
	    		}
	    	}
    	} catch (IOException e){
            e.printStackTrace();
            return;
    	}
    }

    private synchronized void handlePerceptionUpdate(observations_t observations){
    	perceptionObjects = new ArrayList<object_data_t>(Arrays.asList(observations.observations));
    }

    private synchronized void handleBeliefUpdate(soar_objects_t observations){
    	beliefObjects = new ArrayList<object_data_t>(Arrays.asList(observations.objects));
    }

    /** Render ProbCog-specific content. */
    class RenderThread extends Thread
    {
        int fps = 10;

        public void run()
        {
            while (true) {
                if(drawPerceptionObjects){
                	drawPerceptionObjects();
                }
                if(drawBeliefObjects){
                	drawBeliefObjects();
                }
                double[][] faceCamera = calcFaceCameraMatrix();
                if(drawPerceptionObjects){
                	drawPerceptionLabels(faceCamera);
                }
                if(drawBeliefObjects){
                	drawBeliefLabels(faceCamera);
                }

                if(arm != null){
                    arm.render(vw);
                }

                TimeUtil.sleep(1000/fps);

            }
        }
    }

    public double[][] calcFaceCameraMatrix(){
        // === XXX THE BELOW TRIES TO RENDER TEXT OVER OBJECTS ===
    	CameraPosition camera = vl.cameraManager.getCameraTarget();
		double[] forward = LinAlg.normalize(LinAlg.subtract(camera.eye, camera.lookat));
		// Spherical coordinates
        double psi = Math.PI/2.0 - Math.asin(forward[2]);   // psi = Pi/2 - asin(z)
        double theta = Math.atan2(forward[1], forward[0]);  // theta = atan(y/x)
        if(forward[0] == 0 && forward[1] == 0){
        	theta = -Math.PI/2;
        }
        double[][] tilt = LinAlg.rotateX(psi); 				// tilt up or down to face the camera vertically
        double[][] rot = LinAlg.rotateZ(theta + Math.PI/2); // rotate flat to face the camera horizontally
        double[][] faceCamera = LinAlg.matrixAB(rot, tilt);
        return faceCamera;
    }

    private synchronized void drawPerceptionObjects(){
		VisWorld.Buffer buffer = vw.getBuffer("perc-objects");
		for(object_data_t objDat : perceptionObjects){
        	// Draw box outline
            BoundingBox bbox = new BoundingBox(objDat.bbox_dim, objDat.bbox_xyzrpy);
			double[] s = bbox.lenxyz;
			double[][] scale = LinAlg.scale(s[0], s[1], s[2]);
			double[][] trans = LinAlg.xyzrpyToMatrix(bbox.xyzrpy);
			VzBox box = new VzBox(new VzMesh.Style(new Color(0,0,0,0)), new VzLines.Style(Color.blue, 2));
			buffer.addBack(new VisChain(trans, scale, box));
        }
    	buffer.swap();
    }

    private synchronized void drawPerceptionLabels(double[][] faceCamera){
		VisWorld.Buffer buffer = vw.getBuffer("perc-labels");
		for(object_data_t objDat : perceptionObjects){
        	String labelString = "";
    		String tf="<<monospaced,blue,dropshadow=false>>";
    		labelString += String.format("%s%d\n", tf, objDat.id);

    		if(drawPropertyLabels){
        		for(categorized_data_t catDat : objDat.cat_dat){
        			if(catDat.len == 0 || catDat.cat.cat == category_t.CAT_LOCATION){
        				continue;
        			}
        			String label = catDat.label[0];
        			double conf = catDat.confidence[0];
            		labelString += String.format("%s%s:%.2f\n", tf, label, conf);
        		}
    		}

    		double[] pos = objDat.bbox_xyzrpy;

    		VzText text = new VzText(labelString);
    		double[] textLoc = new double[]{pos[0], pos[1], pos[2] + .1};
            buffer.addBack(new VisChain(LinAlg.translate(textLoc), faceCamera, LinAlg.scale(.004), text));
		}
        buffer.swap();
    }

    private synchronized void drawBeliefObjects(){
    	VisWorld.Buffer buffer = vw.getBuffer("belief-objects");
		for(object_data_t objDat : beliefObjects){
        	// Draw box outline
            BoundingBox bbox = new BoundingBox(objDat.bbox_dim, objDat.bbox_xyzrpy);
			double[] s = bbox.lenxyz;
			double[][] scale = LinAlg.scale(s[0], s[1], s[2]);
			double[][] trans = LinAlg.xyzrpyToMatrix(bbox.xyzrpy);
			VzBox box = new VzBox(new VzMesh.Style(new Color(0,0,0,0)), new VzLines.Style(Color.black, 4));
			buffer.addBack(new VisChain(trans, scale, box));
		}
    	buffer.swap();
    }

    private synchronized void drawBeliefLabels(double[][] faceCamera){
    	VisWorld.Buffer buffer = vw.getBuffer("belief-labels");
    	for(object_data_t objDat : beliefObjects){
            BoundingBox bbox = new BoundingBox(objDat.bbox_dim, objDat.bbox_xyzrpy);
    		String tf="<<monospaced,black,dropshadow=#FFFFFFFF>>";
    		String labelString = String.format("%s%d\n", tf, objDat.id);
    		VzText text = new VzText(labelString);
    		double[] corner = new double[]{bbox.lenxyz[0]/2, bbox.lenxyz[1]/2, bbox.lenxyz[2]/2, 1};
    		corner = LinAlg.matrixAB(LinAlg.xyzrpyToMatrix(bbox.xyzrpy), corner);
    		buffer.addBack(new VisChain(LinAlg.translate(corner), faceCamera, LinAlg.scale(.004), text));
    	}

    	buffer.swap();
    }

    // XXX END CODE DUMP
    // ======================================

    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Global configuration file");
        opts.addString('w', "world", null, "Simulated world file");
        opts.addBoolean('p', "perception", false, "Show perception objects");
        opts.addBoolean('l', "labels", false, "Show perception labels");
        opts.addBoolean('b', "belief", false, "Show belief objects");
        opts.addBoolean('k', "kinect", false, "Show kinect data");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }

        // Spin up the GUI
        try {
            PerceptionViewer viewer = new PerceptionViewer(opts);
        } catch (IOException ioex) {
            System.err.println("ERR: Error starting Viewer");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}
