package soargroup.mobilesim.commands;

import lcm.lcm.*;

import java.util.*;
import java.io.IOException;

import april.jmat.*;
import april.util.*;

import soargroup.mobilesim.lcmtypes.*;
import magic2.lcmtypes.*;

/** A class to act as a bridge between the old ProbCog interface and
 *  the new, magic2 code in C. Eventually, we'll want to switch to the
 *  appropriate magic2 lcmtypes totally, but for now, this lets us
 *  hook up with the old interface when possible.
 **/
public class Magic2Bridge implements LCMSubscriber, Runnable
{
    LCM lcm = LCM.getSingleton();

    GetOpt opts;

    Object poseLock = new Object();
    double[] lastL2G = new double[3];
    pose_t lastPose;

    public Magic2Bridge(GetOpt opts)
    {
        this.opts = opts;
    }

    public void run()
    {
        lcm.subscribe(opts.getString("soar-cmd-channel"), this);
        lcm.subscribe(opts.getString("magic2-status-channel"), this);
        lcm.subscribe(opts.getString("pose-channel"), this);
        lcm.subscribe(opts.getString("l2g-channel"), this);

        // LCM publishing loop
        robot_info_t robot_info = new robot_info_t();
        while (true) {
            TimeUtil.sleep(100);

            if (lastPose == null)
                continue;

            synchronized (poseLock) {
                robot_info.utime = TimeUtil.utime();
                robot_info.held_object = 0;

                // Pose stuff
                double[] lxyt = LinAlg.quatPosToXYT(lastPose.orientation,
                                                    lastPose.pos);
                double[] gxyt = LinAlg.xytMultiply(lastL2G, lxyt);

                double[][] M = LinAlg.xytToMatrix(gxyt);
                robot_info.xyzrpy = LinAlg.matrixToXyzrpy(M);
            }

            lcm.publish(opts.getString("robot-info-channel"), robot_info);
        }
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ioex) {
            System.err.println("ERR: LCM channel " + channel);
            ioex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
        throws IOException
    {
        if (channel.equals(opts.getString("soar-cmd-channel"))) {
            control_law_t controlLaw = new control_law_t(ins);

            // Forward the control law on to magic2
            robot_control_policy_t rcp = new robot_control_policy_t();

            rcp.utime = controlLaw.utime;
            rcp.name = controlLaw.name;
            rcp.id = controlLaw.id;

            rcp.num_params = controlLaw.num_params;
            rcp.param_names = new String[rcp.num_params];
            rcp.param_values = new String[rcp.num_params];
            for (int i = 0; i < rcp.num_params; i++) {
                rcp.param_names[i] = controlLaw.param_names[i];
                rcp.param_values[i] = controlLaw.param_values[i].value;
            }

            rcp.num_conditions = 1;
            rcp.conditions = new robot_termination_condition_t[rcp.num_conditions];
            rcp.statuses = new robot_control_policy_status_t[rcp.num_conditions];

            robot_termination_condition_t tc = new robot_termination_condition_t();
            tc.utime = rcp.utime;
            tc.name = controlLaw.termination_condition.name;
            tc.num_params = controlLaw.termination_condition.num_params;
            tc.param_names = new String[tc.num_params];
            tc.param_values = new String[tc.num_params];
            for (int i = 0; i < tc.num_params; i++) {
                tc.param_names[i] = controlLaw.termination_condition.param_names[i];
                tc.param_values[i] = controlLaw.termination_condition.param_values[i].value;
            }

            robot_control_policy_status_t status = new robot_control_policy_status_t();
            status.utime = tc.utime;
            status.status = robot_control_policy_status_t.SUCCESS;
            status.id = controlLaw.id;
            status.name = controlLaw.name;

            rcp.conditions[0] = tc;
            rcp.statuses[0] = status;

            lcm.publish(opts.getString("magic2-cmd-channel"), rcp);

            //interpretCommand(controlLaw);
            //sendCommandStatus(controlLaw);
        } else if (channel.startsWith(opts.getString("magic2-status-channel"))) {
            // Grab status messages from magic2 and shove them back to soar
            robot_control_policy_status_t status = new robot_control_policy_status_t(ins);

            control_law_status_t soar_status = new control_law_status_t();
            soar_status.utime = status.utime;
            soar_status.id = status.id;
            soar_status.name = status.name;

            switch (status.status) {
                case robot_control_policy_status_t.SUCCESS:
                    soar_status.status = "SUCCESS";
                    break;
                case robot_control_policy_status_t.RECEIVED:
                    soar_status.status = "RECEIVED";
                    break;
                case robot_control_policy_status_t.FAILURE:
                    soar_status.status = "FAILURE";
                    break;
                case robot_control_policy_status_t.EXECUTING:
                    soar_status.status = "EXECUTING";
                    break;
                default:
                    soar_status.status = "UNKNOWN";
                    break;
            }

            lcm.publish(opts.getString("soar-status-channel"), soar_status);
        } else if (channel.equals(opts.getString("pose-channel"))) {
            pose_t pose = new pose_t(ins);

            synchronized (poseLock) {
                lastPose = pose;
            }
        } else if (channel.equals(opts.getString("l2g-channel"))) {
            lcmdoubles_t l2g = new lcmdoubles_t(ins);

            synchronized (poseLock) {
                System.arraycopy(l2g.data, 0, lastL2G, 0, 3);
            }
        }
    }

    public static void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h',"help",false,"Show usage");
        opts.addString('\0',"soar-cmd-channel","SOAR_COMMAND_TX","Soar command channel");
        opts.addString('\0',"soar-status-channel","SOAR_COMMAND_STATUS_TX","Soar status channel");
        opts.addString('\0',"magic2-cmd-channel","MAGIC2_CONTROL_POLICY","Magic2 command channel");
        opts.addString('\0',"magic2-status-channel","MAGIC2_CONTROL_POLICY_STATUS","Magic2 status channel");
        opts.addString('\0',"pose-channel","POSE","Pose channel");
        opts.addString('\0',"l2g-channel","L2G","L2G channel");
        opts.addString('\0',"robot-info-channel","ROBOT_INFO","Robot info channel");

        if (!opts.parse(args)) {
            System.out.println("ERR: options - "+opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        new Thread(new Magic2Bridge(opts)).run();
    }
}
