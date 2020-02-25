package soargroup.mobilesim;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.BufferedImage;
import java.io.*;

import javax.swing.*;

import java.text.*;
import java.util.*;
import java.util.Timer;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.util.*;
import april.vis.*;
import april.vis.VisCameraManager.CameraPosition;

import soargroup.mobilesim.MobileSimulator;
import soargroup.mobilesim.CommandSpoofer;

// LCM Types
import lcm.lcm.*;
import soargroup.mobilesim.lcmtypes.rosie_agent_command_t;

public class MobileGUI extends JFrame
{
	public static class Settings {
		public final static boolean PRINT_IDS = false;
		public final static boolean DRAW_ANCHORS = false;
		public final static boolean DRAW_REGION_IDS = false;
		public final static boolean DRAW_LABELS = true;
		public final static boolean LARGE_LABELS = false;
		public final static boolean TELEPORT_ROBOT = false;
		public final static boolean COLLIDE_WALLS = false;
	}

    private MobileSimulator simulator;
    LCM lcm = LCM.getSingleton();

    // Periodic tasks
    PeriodicTasks tasks = new PeriodicTasks(2);

    // Vis Stuff
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    // Parameter Stuff
    ParameterGUI pg;

	PanThread panThread;

    public MobileGUI(GetOpt opts) throws IOException
    {
        super("ProbCog Mobile");
        this.setSize(800, 600);
        this.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        this.setLayout(new BorderLayout());

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        vl.addEventHandler(new MobileGUIEventHandler(vw));
        this.add(vc, BorderLayout.CENTER);

        VisConsole console = new VisConsole(vw, vl, vc);

    	// Initialize the simulator
        // CommandInterpreter ci = new CommandInterpreter();
        simulator = new MobileSimulator(opts, vw, vl, vc, console);

        // Parameter stuff
        pg = new ParameterGUI();
        initParameters();
        this.add(pg, BorderLayout.SOUTH);

        // Set GUI modes
        this.setVisible(true);

        // Render updates about the world
		if(Settings.DRAW_LABELS){
			RenderThread rt = new RenderThread();
			rt.start();
		}

		panThread = new PanThread(vl, vc);
		panThread.start();
    }

    /* PanThread: Handles arrow key presses and pans the screen accordingly */
    class PanThread extends Thread
    {
        int fps = 40;
		private double[] panVel;
		private final double PAN_SPEED = 0.3;
		private VisLayer vl;
		private VisCanvas vc;

        public PanThread(VisLayer vl, VisCanvas vc) {
			this.vl = vl;
			this.vc = vc;
			panVel = new double[]{ 0.0, 0.0, 0.0 };
        }

        public void run() {
            while (true) {
				VisCanvas.RenderInfo rinfo = vc.getLastRenderInfo();
				if(rinfo != null && (panVel[0] != 0.0 || panVel[1] != 0.0)){
					VisCameraManager.CameraPosition cameraPosition = rinfo.cameraPositions.get(vl);
					vl.cameraManager.uiLookAt(LinAlg.add(panVel, cameraPosition.eye),
						LinAlg.add(panVel, cameraPosition.lookat), cameraPosition.up, false);
				}
                TimeUtil.sleep(1000/fps);
            }
        }

		public void startPan(int dim, int dir){
			panVel[dim] = PAN_SPEED * dir;
		}

		public void stopPan(int dim, int dir){
			panVel[dim] = 0.0;
		}

    }

    /** Render ProbCog-specific content. */
    class RenderThread extends Thread implements LCMSubscriber
    {
        int fps = 20;

        public RenderThread()
        {
			// LCM subscriptions
        }

        public void run()
        {
            Tic tic = new Tic();
            while (true) {
                double dt = tic.toctic();
                drawObjectLabels();
                TimeUtil.sleep(1000/fps);
            }
        }
        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            //try {

            //} catch (IOException ex) {
            //    System.err.println("WRN: Error receiving message on channel - "+channel);
            //    ex.printStackTrace();
            //}
        }

		public double[][] calcFaceCameraMatrix(){
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

		public void drawObjectLabels(){
			double[][] faceCamera = calcFaceCameraMatrix();
			VisWorld.Buffer buffer = vw.getBuffer("obj-labels");
			synchronized (simulator) {
				for (SimObject obj: simulator.getWorld().objects) {
					if (!(obj instanceof soargroup.mobilesim.sim.RosieSimObject))
						continue;
					soargroup.mobilesim.sim.RosieSimObject pcobj = (soargroup.mobilesim.sim.RosieSimObject)obj;
					if(!(obj instanceof soargroup.mobilesim.sim.SimPerson)){
						continue;
					}

					String lbl = pcobj.getLabel(Settings.PRINT_IDS);
					lbl = lbl.substring(0, 1).toUpperCase() + lbl.substring(1);
					if(lbl.equals("Co")){
						lbl = "CO";
					}
					String tf="<<monospaced,black,dropshadow=false>>";
					String text = String.format("%s%s\n", tf, lbl);

					VzText vzText = new VzText(text);
					double[] textLoc = new double[]{pcobj.getXYZRPY()[0], pcobj.getXYZRPY()[1], pcobj.getXYZRPY()[2] + 1.5};
					if(Settings.LARGE_LABELS){
						buffer.addBack(new VisChain(LinAlg.translate(textLoc), faceCamera, LinAlg.scale(0.04), vzText));
					} else {
						buffer.addBack(new VisChain(LinAlg.translate(textLoc), faceCamera, LinAlg.scale(0.04), vzText));
					}
				}
			}
			buffer.swap();
		}
    }

	/*************************************************
	 * MobileGUIEventHandler
	 *   Handles GUI input events (mouse/keyboard)
	 ************************************************/

    class MobileGUIEventHandler extends VisEventAdapter
	{
        VisWorld world;

        public MobileGUIEventHandler(VisWorld vw)
        {
            world = vw;
        }

        public int getDispatchOrder()
        {
            return -10000;    // Highest priority
        }

        public boolean keyReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
        {
            if (e.getKeyCode() == KeyEvent.VK_UP){
				panThread.stopPan(1, 1);
			} else if (e.getKeyCode() == KeyEvent.VK_DOWN){
				panThread.stopPan(1, -1);
			} else if (e.getKeyCode() == KeyEvent.VK_RIGHT){
				panThread.stopPan(0, 1);
			} else if (e.getKeyCode() == KeyEvent.VK_LEFT){
				panThread.stopPan(0, -1);
	 		} else {
				return false;
			}

			return true;
        }

        public boolean keyPressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, KeyEvent e)
        {
            if (e.getKeyCode() == KeyEvent.VK_UP){
				panThread.startPan(1, 1);
			} else if (e.getKeyCode() == KeyEvent.VK_DOWN){
				panThread.startPan(1, -1);
			} else if (e.getKeyCode() == KeyEvent.VK_RIGHT){
				panThread.startPan(0, 1);
			} else if (e.getKeyCode() == KeyEvent.VK_LEFT){
				panThread.startPan(0, -1);
	 		} else if (e.getKeyCode() == KeyEvent.VK_SPACE){
				rosie_agent_command_t agent_command = new rosie_agent_command_t();
				agent_command.utime = TimeUtil.utime();
				agent_command.command_type = rosie_agent_command_t.CONTINUE;
                lcm.publish("ROSIE_AGENT_COMMAND", agent_command);
			} else if (e.getKeyCode() == KeyEvent.VK_ESCAPE){
				rosie_agent_command_t agent_command = new rosie_agent_command_t();
				agent_command.utime = TimeUtil.utime();
				agent_command.command_type = rosie_agent_command_t.INTERRUPT;
                lcm.publish("ROSIE_AGENT_COMMAND", agent_command);
			} else {
				return false;
			}

			return true;
        }

        public boolean mouseMoved(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
			// draws mouse world coordinates in the corner
            double[] xy = ray.intersectPlaneXY();
            Formatter f = new Formatter();
            f.format("<<monospaced-128>>(%.2f, %.2f)", xy[0], xy[1]);
            VisWorld.Buffer vb = world.getBuffer("coordinates");

            vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_RIGHT,
                                        LinAlg.scale(0.1),
                                        new VzText(VzText.ANCHOR.BOTTOM_RIGHT_ROUND,
                                                   f.toString())));
            vb.swap();
            return false;
        }
    }

	/*************************************************
	 * Parameters
	 *   Way to set parameters which can be changed by the user in the GUI
	 ************************************************/

    private void initParameters()
    {
        // pg.addCheckBoxes("param-name", "Param Name", initValue);

        pg.addListener(new ParameterListener(){
			public void parameterChanged(ParameterGUI pg, String name) {
				if(name.equals("param-name")){
					// use parameter value pg.gb(name)
				}
			}
		});
    }

    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('w', "world", null, "Simulated world file");
        opts.addBoolean('s', "spoof", false, "Open small GUI to spoof soar commands");
		opts.addBoolean('f', "fully", false, "Whether a room is fully observable");

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
            MobileGUI sim = new MobileGUI(opts);
            if(opts.getBoolean("spoof")) {
                CommandSpoofer spoof = new CommandSpoofer();
            }
        } catch (IOException ioex) {
            System.err.println("ERR: Error starting GUI");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}
