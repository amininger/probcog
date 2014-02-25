package probcog.arm;

import java.awt.Color;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.lcmtypes.*;

/** This class listens in on arm status messages and
 *  uses them, in conjuction with a config file specifying
 *  parameters of the arm, convey information about the arm
 *  to the user.
 *
 *  Unlike BoltArm, this is not a singleton object. Any
 *  process interested in the arm creates its own instance
 *  of ArmStatus to track the state of the arm.
 */
public class ArmStatus implements LCMSubscriber
{
    private LCM lcm = LCM.getSingleton();

    Config config;
    String prefix;

    private ArrayList<Joint> joints = new ArrayList<Joint>();
    private ArrayList<Double> armWidths = new ArrayList<Double>();
    public static double baseHeight = 0.0;
    public static double wristHeight = 0.0;

    private ExpiringMessageCache<dynamixel_status_list_t> statuses =
        new ExpiringMessageCache<dynamixel_status_list_t>(1.0, true);
    private ExpiringMessageCache<dynamixel_command_list_t> commands =
        new ExpiringMessageCache<dynamixel_command_list_t>(1.0, true);

    public ArmStatus(Config config_) throws IOException
    {
        this(config_, "ARM");
    }

    /** Takes a channel prefix as an argument to identify which arm this
     *  observer should listen in on.
     */
    public ArmStatus(Config config_, String prefix_) throws IOException
    {
        // Construct config
        config = new ConfigFile(config_.getPath("robot.arm"));
        prefix = prefix_;

        initArm();

        lcm.subscribe(prefix+"_STATUS", this);
        lcm.subscribe("ARM_COMMAND", this); // XXX Multi arm support!
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ioex) {
            System.err.println("ERR: LCM channel "+channel);
            ioex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
        throws IOException
    {
        if (channel.equals(prefix+"_STATUS")) {
            // Handle arm status updates. Saves the status message for
            // consumption by user in addition to updating the arm model
            dynamixel_status_list_t dsl = new dynamixel_status_list_t(ins);
            long utime = Long.MAX_VALUE;
            for (dynamixel_status_t s: dsl.statuses) {
                utime = Math.min(utime, s.utime);
            }

            if (statuses.put(dsl, utime)) {
                for (int i = 0; i < dsl.len; i++) {
                    joints.get(i).updatePos(dsl.statuses[i].position_radians);
                }
            }
        } else if (channel.equals("ARM_COMMAND")) {
            dynamixel_command_list_t dcl = new dynamixel_command_list_t(ins);
            long utime = Long.MAX_VALUE;
            for (dynamixel_command_t c: dcl.commands) {
                utime = Math.min(utime, c.utime);
            }

            commands.put(dcl, utime);
        }
    }

    /** Creates the initial internal model of the arm. Later, this model
     *  will be used to provide positional data about the arm's joints,
     *  etc.
     */
    private void initArm()
    {
        joints.clear();
        armWidths.clear();
        Joint j0, j1, j2, j3, j4, j5;
        RevoluteJoint.Parameters p0, p1, p2, p3, p4;

        String name = config.getString("arm.arm_version", null);
        assert (name != null);

        baseHeight = config.getDouble("arm."+name+".base_height", 0);
        wristHeight = config.getDouble("arm."+name+".wrist_height",0);
        armWidths.add(0.055);

        for (int i = 0;; i++) {
            double[] range = config.getDoubles("arm."+name+".r"+i+".range", null);
            double length = config.getDouble("arm."+name+".r"+i+".length", 0);
            String axis = config.getString("arm."+name+".r"+i+".axis", null);
            double speed = config.getDouble("arm."+name+".r"+i+".speed", 0);
            double torque = config.getDouble("arm."+name+".r"+i+".torque", 0);
            double width = config.getDouble("arm."+name+".r"+i+".width", 0);
            if (range == null)
                break;

            RevoluteJoint.Parameters params = new RevoluteJoint.Parameters();
            params.lSegment = length;
            params.rMin = Math.toRadians(range[0]);
            params.rMax = Math.toRadians(range[1]);
            if (axis.equals("X")) {
                params.orientation = RevoluteJoint.X_AXIS;
            } else if (axis.equals("Y")) {
                params.orientation = RevoluteJoint.Y_AXIS;
            } else if (axis.equals("Z")) {
                params.orientation = RevoluteJoint.Z_AXIS;
            } else {
                System.err.println("ERR: Bad axis specification - "+axis);
                System.err.println("Defaulting to Y-axis rotation");
                params.orientation = RevoluteJoint.Y_AXIS;
            }

            params.speed = speed;
            params.torque = torque;

            joints.add(new RevoluteJoint(params));
            armWidths.add(width);

        }
        joints.add(new HandJoint(new HandJoint.Parameters()));
        armWidths.add(.05);
    }

    // === User interface to manipulating the arm ===
    public int getNumJoints()
    {
        return joints.size();
    }

    /** Returns the most recent status of the requested servo */
    public dynamixel_status_t getStatus(int idx)
    {
        dynamixel_status_list_t dsl = statuses.get();
        if (dsl == null)
            return null;
        if (idx >= dsl.len)
            return null;
        return dsl.statuses[idx];
    }

    public dynamixel_command_t getMostRecentCommand(int idx)
    {
        dynamixel_command_list_t dcl = commands.get();
        if (dcl == null)
            return null;
        if (idx >= dcl.len)
            return null;
        return dcl.commands[idx];
    }

    /** Returns the desired position of the requested servo */
    public double getDesiredPos(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getDesiredValue();
    }

    /** Returns the actual position of the requested servo */
    public double getActualPos(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getActualValue();
    }

    /** Get the length of segment idx */
    public double getLength(int idx)
    {
        if (idx >= joints.size())
            return -1;
        return joints.get(idx).getLength();
    }

    public double getMinValue(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getMinValue();
    }

    public double getMaxValue(int idx)
    {
        if (idx >= joints.size())
            return Double.MAX_VALUE;
        return joints.get(idx).getMaxValue();
    }

    public dynamixel_command_t getCommand(int idx)
    {
        if (idx >= joints.size())
            return null;
        return joints.get(idx).getArmCommand();
    }

    /** Set the position of servo idx to pos */
    public void setPos(int idx, double pos)
    {
        if (idx >= joints.size())
            return;
        joints.get(idx).setPos(pos);
    }

    // ==============================================

    /** Get the shape of the arm's gripper */
    public Shape getGripperShape()
    {
        return ((HandJoint)(joints.get(getNumJoints()-1))).getShape();
    }

    /** Get the position of the mobile fingers */
    public double[][] getFingerPose()
    {
        double[][] xform = getPoseAt(getNumJoints()-2);
        LinAlg.timesEquals(xform,
                           ((HandJoint)(joints.get(getNumJoints()-1))).getMobileFingerPose());
        return xform;
    }

    public double[][] getPoseAt(int joint)
    {
        double[][] xform = LinAlg.translate(0,0,baseHeight);
        for (int i = 0; i <= Math.min(joint, joints.size()-1); i++) {
            LinAlg.timesEquals(xform, joints.get(i).getRotation());
            LinAlg.timesEquals(xform, joints.get(i).getTranslation());
        }

        return xform;
    }

    /** Get the current position of the arm's gripper */
    public double[][] getGripperPose()
    {
        return getPoseAt(joints.size()-1);
    }

    /** Get the position of the gripper */
    public double[] getGripperXYZRPY()
    {
        return LinAlg.matrixToXyzrpy(getGripperPose());
    }

    /** Return a list of points designating the endpoints of arm segments
     *  along the arm based on the current joint settings.
     */
    public ArrayList<double[]> getArmPoints()
    {
        ArrayList<double[]> points = new ArrayList<double[]>();
        points.add(new double[3]);
        points.add(new double[]{0,0,baseHeight});
        double[][] xform = LinAlg.translate(0,0,baseHeight);
        for (Joint j: joints) {
            LinAlg.timesEquals(xform, j.getRotation());
            LinAlg.timesEquals(xform, j.getTranslation());
            points.add(LinAlg.resize(LinAlg.matrixToXyzrpy(xform),3));
        }

        return points;
    }

    /** Return a list of arm segments widths as specifed by the config file */
    public ArrayList<Double> getArmSegmentWidths()
    {
        return armWidths;
    }

    /** Render the arm in the given VisWorld */
    public void render(VisWorld vw)
    {
        VisWorld.Buffer vb = vw.getBuffer("arm");

        // Render a base (XXX Update me)
        vb.addBack(new VisChain(LinAlg.rotateZ(-Math.PI/2),
                                new  VzTriangle(0.08, 0.08, 0.08,
                                                new VzMesh.Style(Color.green))));
        vb.addBack(new VisChain(LinAlg.translate(0, 0, baseHeight/2),
                                new VzBox(0.04, 0.04, baseHeight,
                                          new VzMesh.Style(Color.black))));

        // Render the joints
        double[][] xform = LinAlg.translate(0, 0, baseHeight);
        for (Joint j: joints) {
            LinAlg.timesEquals(xform, j.getRotation());
            vb.addBack(new VisChain(LinAlg.copy(xform),
                                    j.getVis()));
            LinAlg.timesEquals(xform, j.getTranslation());
        }
        vb.swap();

        /*
        vb = vw.getBuffer("orientation");
        vb.addBack(new VisChain(getGripperPose(),
                                LinAlg.scale(0.1),
                                new VzAxes()));

        vb.swap();
        */
    }
}
