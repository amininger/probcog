package probcog.commands;

import lcm.lcm.*;

import java.util.*;
import java.io.IOException;

import april.util.*;

import probcog.lcmtypes.*;
import magic2.lcmtypes.*;

/** A class to act as a bridge between the old ProbCog interface and
 *  the new, magic2 code in C. Eventually, we'll want to switch to the
 *  appropriate magic2 lcmtypes totally, but for now, this lets us
 *  hook up with the old interface when possible.
 **/
public class Magic2Bridge implements LCMSubscriber
{
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
            if (channel.startsWith("SOAR_COMMAND") && !channel.startsWith("SOAR_COMMAND_STATUS")) {
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

                rcp.conditions[0] = tc;
                rcp.statuses[0] = status;

                lcm.publish("MAGIC2_CONTROL_POLICY", rcp);

                //interpretCommand(controlLaw);
                //sendCommandStatus(controlLaw);
            } else if (channel.startsWith("MAGIC2_CONTROL_POLICY_STATUS")) {
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

                lcm.publish("SOAR_COMMAND_STATUS_TX", soar_status);
            }
        }
}
