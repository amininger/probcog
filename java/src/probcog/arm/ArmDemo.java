package probcog.arm;

import java.awt.*;
import java.awt.event.*;
import javax.swing.*;
import java.util.*;
import java.io.*;

import lcm.lcm.*;

import april.config.*;
import april.dynamixel.*;
import april.jserial.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.*;
import probcog.perception.*;

public class ArmDemo implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();
    ArmStatus arm;
    Tracker tracker;

    // Rendering thread
    RenderThread rt;
    ActionState action = ActionState.UNKNOWN;

    ExpiringMessageCache<observations_t> observations = new ExpiringMessageCache<observations_t>(2.5, true);

    public ArmDemo(Config config_, Tracker tracker_) throws IOException
    {
        arm = new ArmStatus(config_);
        tracker = tracker_;

        lcm.subscribe("OBSERVATIONS", this);

        rt = new RenderThread();
        rt.start();
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("ERR: Could not handle LCM channel - "+channel);
            ex.printStackTrace();
        }
    }

    // XXX Highlight if/for etc stuff
    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("OBSERVATIONS")) {
            // Place observations on the map
            observations_t obs = new observations_t(ins);
            observations.put(obs, obs.utime);
        }
    }

    static enum ActionState
    {
        POINT, GRAB, DROP, SWEEP, RESET, UNKNOWN
    }

    class RenderThread extends Thread
    {
        int fps = 5;

        VisWorld vw;
        VisLayer vl;
        VisCanvas vc;

        double[] xyz = null;

        public RenderThread()
        {
            JFrame jf = new JFrame("Arm Simulation Demo");
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setLayout(new BorderLayout());
            jf.setSize(800, 600);

            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);
            jf.add(vc, BorderLayout.CENTER);

            // ParameterGUI
            ParameterGUI pg = new ParameterGUI();
            pg.addButtons("reset", "Reset");
            jf.add(pg, BorderLayout.SOUTH);

            pg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name) {
                    if (name.equals("reset")) {
                        lcm.publish("ROBOT_COMMAND", getRobotCommand(null, ActionState.RESET));
                    }
                }
            });

            ArmStatusPanel statusPanel = new ArmStatusPanel();
            jf.add(statusPanel, BorderLayout.EAST);

            // Add a classification debug bar
            ParameterGUI classypg = new ParameterGUI();
            classypg.addString("color", "Color", "red");
            classypg.addString("shape", "Shape", "square");
            classypg.addString("size", "Size", "small");
            classypg.addButtons("b_color", "Set Color");
            classypg.addButtons("b_shape", "Set Shape");
            classypg.addButtons("b_size", "Set Size");
            classypg.addListener(new ParameterListener() {
                public void parameterChanged(ParameterGUI pg, String name) {
                    String label = null;
                    FeatureCategory cat = null;
                    if (name.equals("b_color")) {
                        label = pg.gs("color");
                        cat = FeatureCategory.COLOR;
                    } else if (name.equals("b_shape")) {
                        label = pg.gs("shape");
                        cat = FeatureCategory.SHAPE;
                    } else if (name.equals("b_size")) {
                        label = pg.gs("size");
                        cat = FeatureCategory.SIZE;
                    } else {
                        return;
                    }

                    // Find the object
                    observations_t obs = observations.get();
                    if (obs == null) {
                        return;
                    }
                    int id = obs.click_id;  // ID of the currently clicked object
                    HashMap<Integer, Obj> state = tracker.getWorldState();
                    Obj obj;
                    synchronized (state) {
                        obj = state.get(id);
                    }
                    if (obj == null) {
                        System.err.println("ERR: No object with ID = "+id);
                        return;
                    }

                    System.out.println("Added training - "+label);
                    tracker.addTraining(cat, obj.getFeatures(cat), label);
                }
            });
            jf.add(classypg, BorderLayout.WEST);

            // Grid
            VzGrid.addGrid(vw);

            // Board outlines
            VisWorld.Buffer vbBoards = vw.getBuffer("boards");
            double f2m = 0.3048;
            vbBoards.addBack(new VzRectangle(2*f2m, 2*f2m, new VzLines.Style(Color.cyan, 2)));
            vbBoards.addBack(new VzRectangle(3*f2m, 3*f2m, new VzLines.Style(Color.white, 2)));
            vbBoards.swap();

            // Default zoom
            vl.cameraManager.fit2D(new double[] {-1,-1}, new double[] {1,1}, true);

            // Event handler
            vl.addEventHandler(new GoalEventHandler());

            jf.setVisible(true);
        }

        public void run()
        {
            while (true) {
                // Render Arm
                arm.render(vw);

                // Draw current goal
                {
                    VisWorld.Buffer vb = vw.getBuffer("goal");
                    Color color;
                    switch (action) {
                        case POINT:
                            color = new Color(0xff6699);
                            break;
                        case GRAB:
                            color = new Color(0x00cc00);
                            break;
                        case DROP:
                            color = new Color(0x005500);
                            break;
                        case SWEEP:
                            color = new Color(0x00ffff);
                            break;
                        case RESET:
                            color = new Color(0xff0000);
                            break;
                        default:
                            color = new Color(0xffffff);
                            break;
                    }
                    if (xyz != null) {
                        vb.addBack(new VisChain(LinAlg.translate(xyz),
                                                new VzCircle(0.015, new VzMesh.Style(color))));
                    }
                    vb.swap();
                }

                // Draw the currently observed objects w/IDs
                {
                    VisWorld.Buffer vb = vw.getBuffer("observations");
                    observations_t obs = observations.get();
                    if (obs != null) {
                        for (object_data_t od : obs.observations) {
                            Color color = Color.cyan;
                            for (categorized_data_t cat_data: od.cat_dat) {
                                if (cat_data.cat.cat != category_t.CAT_COLOR)
                                    continue;
                                if(cat_data.len > 0){
                                    String label = cat_data.label[0];
                                    if (label.contains("red")) {
                                        color = Color.red;
                                    } else if (label.contains("orange")) {
                                        color = Color.orange;
                                    } else if (label.contains("yellow")) {
                                        color = Color.yellow;
                                    } else if (label.contains("green")) {
                                        color = Color.green;
                                    } else if (label.contains("blue")) {
                                        color = Color.blue;
                                    } else if (label.contains("purple")) {
                                        color = Color.magenta;
                                    } else if (label.contains("black")) {
                                        color = Color.black;
                                    }
                                }
                                else{
                                    color = Color.gray;
                                }
                            }
                            if (color.equals(Color.black))
                                continue;
                            Formatter f = new Formatter();
                            f.format("ID: %d", od.id);
                            double[] obj_xyz = LinAlg.resize(od.pos, 3);
                            vb.addBack(new VisChain(LinAlg.translate(obj_xyz),
                                                    LinAlg.scale(0.02),
                                                    new VzSphere(new VzMesh.Style(color))));
                            vb.addBack(new VisChain(LinAlg.translate(obj_xyz),
                                                    LinAlg.scale(0.002),
                                                    new VzText(f.toString())));

                        }
                    vb.swap();
                    }
                }

                TimeUtil.sleep(1000/fps);
            }
        }

        /** Set the arm goal point */
        public void setGoal(double[] goal)
        {
            xyz = LinAlg.copy(goal);
        }
    }

    class GoalEventHandler extends VisEventAdapter
    {
        public int getDispatchOrder()
        {
            return 0;
        }

        public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] xy = LinAlg.resize(ray.intersectPlaneXY(), 2);
            int mods = e.getModifiersEx();
            boolean shift = (mods & MouseEvent.SHIFT_DOWN_MASK) > 0;
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) > 0;
            observations_t obs = observations.get();
            double minDist = Double.MAX_VALUE;
            double maxSelectionDistance = 0.075;
            int id = 0;
            double[] objPos = null;
            if (obs != null) {
                for (object_data_t obj_dat : obs.observations) {
                    double[] pos = LinAlg.resize(obj_dat.pos, 2);
                    double mag = LinAlg.distance(pos, xy);
                    if (mag < minDist && mag < maxSelectionDistance) {
                        minDist = mag;
                        id = obj_dat.id;
                        objPos = pos;
                    }
                }
            }

            if (shift && !ctrl) {
                if (minDist < 0.05) {
                    lcm.publish("ROBOT_COMMAND", getRobotCommand(id, ActionState.POINT));
                    rt.setGoal(objPos);
                } else {
                    lcm.publish("ROBOT_COMMAND", getRobotCommand(xy, ActionState.POINT));
                    rt.setGoal(xy);
                }
                return true;
            } else if (!shift && ctrl) {
                lcm.publish("ROBOT_COMMAND", getRobotCommand(xy, ActionState.DROP));
                rt.setGoal(xy);
                return true;
            } else if (shift && ctrl) {
                if (minDist != Double.MAX_VALUE) {
                    lcm.publish("ROBOT_COMMAND", getRobotCommand(id, ActionState.GRAB));
                    rt.setGoal(objPos);
                    return true;
                }
            }

            return false;
        }

        public boolean mouseDragged(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            double[] xyz = ray.intersectPlaneXY();
            int mods = e.getModifiersEx();
            boolean shift = (mods & MouseEvent.SHIFT_DOWN_MASK) > 0;
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) > 0;

            return false;
        }
    }

    private robot_command_t getRobotCommand(int id, ActionState state)
    {
        action = state;
        robot_command_t cmd = new robot_command_t();
        cmd.utime = TimeUtil.utime();
        cmd.updateDest = false;
        cmd.dest = new double[6];
        switch (state) {
            case POINT:
                cmd.action = "POINT="+id;
                break;
            case GRAB:
                cmd.action = "GRAB="+id;
                break;
            case SWEEP:
                cmd.action = "SWEEP="+id;
                break;
            case DROP:
                cmd.action = "DROP="+id;
                break;
            default:
                cmd.action = "RESET="+id;
                break;
        }

        return cmd;
    }

    private robot_command_t getRobotCommand(double[] dest, ActionState state)
    {
        action = state;
        robot_command_t cmd = new robot_command_t();
        cmd.utime = TimeUtil.utime();
        cmd.updateDest = (dest != null);
        cmd.dest = (dest == null) ? new double[6] : LinAlg.resize(dest, 6);
        switch (state) {
            case POINT:
                cmd.action = "POINT";
                break;
            case GRAB:
                cmd.action = "GRAB";
                break;
            case SWEEP:
                cmd.action = "SWEEP";
                break;
            case DROP:
                cmd.action = "DROP";
                break;
            default:
                cmd.action = "RESET";
                break;
        }

        return cmd;
    }


    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addString('c',"config",null,"Config file");
        opts.addString('w',"world",null,"Sim world file");
        //opts.addBoolean('s',"sim",false,"Run in simulation mode");
        opts.addBoolean('h',"help",false,"Display this help screen");

        if (!opts.parse(args)) {
            System.err.println("ERR: Option error - "+opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }

        if (opts.getString("config") == null) {
            System.err.println("ERR: Need to supply config file");
            System.exit(1);
        }

        Config config = null;
        try {
            config = new ConfigFile(opts.getString("config"));
        } catch (IOException ioex) {
            System.err.println("ERR: Unable to open config file");
            ioex.printStackTrace();
            System.exit(1);
        }


        try {
            // Take the presence of a world file as a sign that we want a sim
            Tracker tracker;
            if (opts.getString("world") == null) {
                System.out.println("Spinning up real world...");
                tracker = new Tracker(config, true, false, null);
                ArmDriver driver = new ArmDriver(config);
                (new Thread(driver)).start();
            } else {
                System.out.println("Spinning up simulation...");
                SimWorld world = new SimWorld(opts.getString("world"),
                                              new Config());
                tracker = new Tracker(config, false, false, world);
                SimArm simArm = new SimArm(config, world);
            }
            ArmController controller = new ArmController(config);
            ArmDemo demo = new ArmDemo(config, tracker);
        } catch (IOException ioex) {
            System.err.println("ERR: Could not start up arm debugging interface");
            ioex.printStackTrace();
        }
    }
}
