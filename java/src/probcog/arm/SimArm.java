package probcog.arm;

import java.io.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.util.*;

// import probcog.sim.*;
import probcog.lcmtypes.*;
import probcog.sim.SimObjectPC;

/** Simulates the movements of the robotic arm through the environment based
 *  on observed arm commands. Produces arm_status_t messages on the designated
 *  channel for observation.
 */
public class SimArm implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    // === Servo Parameters ===
    // Note that, without taking torque into account, it is unlikely that we
    // can make a resonable estimate at achieved speeds, but we will act like
    // we achieve max speed and move precisely.
    static final int DYNAMIXEL_MAX_SPEED = 1023;
    static final double DYNAMIXEL_SPEED_INC = 0.114 * Math.PI / 30; // [rad/s]

    Config config;
    SimWorld simWorld;
    BoxShape groundPlane;
    double[][] planePose = LinAlg.identity(4);
    String prefix;

    // Our model of the arm
    ArmStatus arm;
    ExpiringMessageCache<dynamixel_command_list_t> cmdCache =
        new ExpiringMessageCache<dynamixel_command_list_t>(0.25);

    SimObject grabbed = null;

    // In the ideal world, we would actually have a better understanding of
    // which arm we were modeling and how it was constructed such that we could
    // actually roughly model the torques exerted on the arm, etc. This is too
    // detailed a model for our immediate needs, so we'll simply model speed
    // based on commands given.
    //
    // Object grabbing will observe the positions of objects in the environment
    // (simulated only) to spoof appropriate statuses indicating
    // when an object has been "grabbed" so that picking up/putting down may
    // occur
    public SimArm(Config config_, SimWorld sw_) throws IOException
    {
        this(config_, sw_, "ARM");  // Will not place nice for more than 1 arm
    }

    public SimArm(Config config_, SimWorld sw_, String prefix_) throws IOException
    {
        config = config_;
        simWorld = sw_;
        prefix = prefix_;

        // Construct a large ground plane comparable to our arm play area
        groundPlane = new BoxShape(.9144, .9144, 0.001);

        arm = new ArmStatus(config_, prefix_);

        lcm.subscribe(prefix+"_COMMAND", this);

        (new SimulationThread()).start();
    }

    class SimulationThread extends Thread
    {
        int Hz = 15;
        int maxDelay = (int)(1000.0/Hz);    // [ms]
        double dt = 1.0/Hz;

        double[][] lastPose = arm.getGripperPose();
        double[][] deltaGrabbed = null;

        public void run()
        {
            System.out.println("ATTN: Starting simulation thread");
            while (true) {
                long utime = TimeUtil.utime();

                dynamixel_command_list_t cmds;
                synchronized (this) {
                    cmds = cmdCache.get();
                }

                // Create a status moving us towards the goal position
                dynamixel_status_list_t dsl = new dynamixel_status_list_t();
                dsl.len = arm.getNumJoints();
                dsl.statuses = new dynamixel_status_t[dsl.len];
                for (int i = 0; i < dsl.len; i++) {
                    dynamixel_status_t status = new dynamixel_status_t();
                    status.utime = utime;

                    // Default values
                    status.speed = 0;
                    status.position_radians = arm.getActualPos(i);

                    // Load
                    //
                    // Needed for grabbing. Grabbing will want a load around
                    // .275 to .4 to consider a grip "stable"
                    status.load = 0;

                    // Temperature
                    status.temperature = 0; // XXX Unused

                    // Voltage
                    status.voltage = 0; // XXX Unused


                    // Handle commands
                    if (cmds != null) {
                        dynamixel_command_t cmd = cmds.commands[i];
                        //cmd.speed = 1.0; // DEBUG XXX
                        // Rotation
                        double pos = arm.getActualPos(i);
                        int sign = pos <= cmd.position_radians ? 1 : -1;
                        double dr = cmd.speed*DYNAMIXEL_MAX_SPEED*DYNAMIXEL_SPEED_INC;
                        if (sign > 0)  {
                            status.position_radians = Math.min(pos + dr*dt,
                                                               cmd.position_radians);
                        } else {
                            status.position_radians = Math.max(pos - dr*dt,
                                                               cmd.position_radians);
                        }

                        if (i == 5 && grabbed != null) {
                            status.position_radians = pos;
                        }

                        // Speed
                        boolean stopped = status.position_radians == pos;
                        if (!stopped) {
                            status.speed = cmd.speed;
                        }
                        //System.out.println("SPEED " + i + " == " + status.speed);

                        // Grabbing
                        // If the hand joint is moving in the positive direction,
                        // check for object collisions and try to grab objects.
                        // Otherwise, if the joint is opening and we're not
                        // holding something, we should drop our object
                        if (i == 5 && sign > 0 && !stopped && grabbed == null) {
                            for (SimObject so: simWorld.objects) {
                                if (Collisions.collision(so.getShape(),
                                                         so.getPose(),
                                                         arm.getGripperShape(),
                                                         arm.getFingerPose()))
                                {
                                    if (grabbed != null) {
                                        double[] curxyzrpy = LinAlg.matrixToXyzrpy(grabbed.getPose());
                                        double[] newxyzrpy = LinAlg.matrixToXyzrpy(so.getPose());
                                        if (curxyzrpy[2] > newxyzrpy[2])
                                            continue;
                                    }
                                    grabbed = so;
                                    double[][] A = arm.getGripperPose();
                                    double[][] S = so.getPose();
                                    deltaGrabbed = LinAlg.matrixAB(LinAlg.inverse(A),
                                                                   S);
                                }
                            }
                        } else if (i == 5 && sign < 0 && grabbed != null) {
                            // Drop the object, if we're holding one
                            // Insta-drop. Move object down step-by-step
                            // while checking for collision with other
                            // objects AND the ground plane. When contact
                            // is made, stop.
                            double dropStep = 0.005;
                            while (true) {
                                boolean contact = false;
                                for (SimObject so: simWorld.objects) {
                                    // No self collisions
                                    if (so == grabbed)
                                        continue;

                                    if (Collisions.collision(so.getShape(),
                                                             so.getPose(),
                                                             grabbed.getShape(),
                                                             grabbed.getPose()))
                                    {
                                        contact = true;
                                        break;
                                    }
                                }

                                if (contact ||
                                    Collisions.collision(groundPlane,
                                                         planePose,
                                                         grabbed.getShape(),
                                                         grabbed.getPose()))
                                {
                                    break;
                                }

                                double[][] pose = grabbed.getPose();
                                double[] xyzrpy = LinAlg.matrixToXyzrpy(pose);
                                xyzrpy[2] -= dropStep;
                                if (xyzrpy[2] < 0) {
                                    break;
                                }

                                pose = LinAlg.xyzrpyToMatrix(xyzrpy);
                                synchronized (simWorld) {
                                    grabbed.setPose(pose);
                                }
                            }

                            // XXX: AM: Hack so that grabbed objects aren't viewed (only in perfect segmentation)
                            if(grabbed instanceof SimObjectPC){
                            	((SimObjectPC)grabbed).setVisible(true);
                            }
                            grabbed = null;
                        }

                        if (grabbed != null && i == 5) {
                            status.load = 0.3;
                        }
                    }


                    // Don't simulate error codes
                    status.error_flags = 0;

                    // Uncomment to debug flags
                    //status.error_flags = dynamixel_status_t.ERROR_VOLTAGE |
                    //                     dynamixel_status_t.ERROR_OVERLOAD |
                    //                     dynamixel_status_t.ERROR_ANGLE_LIMIT |
                    //                     dynamixel_status_t.ERROR_OVERHEAT;



                    dsl.statuses[i] = status;
                }

                synchronized (simWorld) {
                    if (grabbed != null) {
                        double[][] currPose = arm.getGripperPose();

                        double[][] objPose = LinAlg.matrixAB(currPose, deltaGrabbed);
                        
                        // XXX: AM: Hack so sim objects don't get rotated except along yaw
                        double[] xyzrpy = LinAlg.matrixToXyzrpy(objPose);
                        xyzrpy[3] = 0;
                        xyzrpy[4] = 0;
                        grabbed.setPose(LinAlg.xyzrpyToMatrix(xyzrpy));
                        
                        // XXX: AM: Hack so that grabbed objects aren't viewed (only in perfect segmentation)
                        if(grabbed instanceof SimObjectPC){
                        	((SimObjectPC)grabbed).setVisible(false);
                        }
                    }
                }
                lastPose = arm.getGripperPose();

                lcm.publish(prefix+"_STATUS", dsl);

                long now = TimeUtil.utime();
                int delay = Math.min(maxDelay, (int)((now - utime)/1000.0));

                TimeUtil.sleep(maxDelay - delay);
            }
        }
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ioex) {
            System.err.println("ERR: Couldn't handle LCM channel - "+channel);
            ioex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
        throws IOException
    {
        if (channel.equals(prefix+"_COMMAND")) {
            dynamixel_command_list_t cmds = new dynamixel_command_list_t(ins);
            if (cmds.len != 6) {
                System.err.println("ERR: Invalid command length received");
            } else {
                synchronized (this) {
                    cmdCache.put(cmds, TimeUtil.utime());
                }
            }
        }
    }

    static public void main(String[] args)
    {
        // XXX Start up a simulation
    }
}
