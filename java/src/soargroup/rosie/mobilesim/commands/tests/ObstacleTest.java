package soargroup.rosie.mobilesim.commands.tests;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;
import april.util.*;

import soargroup.rosie.mobilesim.commands.*;
import soargroup.rosie.mobilesim.util.*;

import magic2.lcmtypes.*;

/** A condition test that fails when the robot is sufficiently close to running
 *  into an obstacle. This is a last-ditch safety tool, typically, though could
 *  also be used for "follow the wall until you reach a dead end" type things.
 **/
public class ObstacleTest implements ConditionTest, LCMSubscriber
{
    LCM lcm = LCM.getSingleton();
    String laserChannel = Util.getConfig().getString("robot.lcm.laser_channel", "LASER");

    // Stay at least half a meter away from things directly ahead of you
    double hazardDistance = 0.5;
    boolean danger = false;

    public ObstacleTest()
    {
    }

    public ObstacleTest(HashMap<String, TypedValue> parameters)
    {
        if (parameters.containsKey("distance"))
            hazardDistance = parameters.get("distance").getDouble();
    }

    public ConditionTest copyCondition()
    {
        return null;
    }

    /** Query whether or not the condition being tested for is currently true.
     *
     *  @return True if condition test is currently satisfied, else false
     **/
    public boolean conditionMet()
    {
        return danger;
    }

    /** Get the parameters that can be set for this condition test.
     *
     *  @return An iterable collection of all possible parameters.
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();
        params.add(new TypedParameter("distance",
                                      TypedValue.TYPE_DOUBLE,
                                      new TypedValue((double)0.0),
                                      new TypedValue((double)5.0),
                                      false));

        return params;
    }

    public void setRunning(boolean run)
    {
        if (run) {
            lcm.subscribe(laserChannel, this);
        } else {
            lcm.unsubscribe(laserChannel, this);
        }
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (danger)
                return;

            if (laserChannel.equals(channel)) {
                laser_t laser = new laser_t(ins);
                double[] xAxis = new double[] {1.0, 0};
                double[] yAxis = new double[] {0, 1.0};

                // Determine if anything is hazardous in front of the robot
                for (int i = 0; i < laser.nranges; i++) {
                    double r = laser.ranges[i];
                    if (r < 0)
                        continue;
                    double t = laser.rad0 + i*laser.radstep;
                    double[] xy = new double[] {r*Math.cos(t), r*Math.sin(t)};

                    // Is the point ahead of the robot and within our hazard distance?
                    // If yes, alert the system to DANGER!
                    double width = Math.abs(LinAlg.dotProduct(xy, yAxis));
                    if (width > .3)
                        continue;   // Too far out from robot

                    double dist = LinAlg.dotProduct(xy, xAxis);
                    if (dist >= 0 && dist < hazardDistance) {
                        danger = true;
                        return;
                    }
                }
            }

        } catch (IOException ex) {
            System.err.println("ERR: Could not handle message on channel "+channel);
            ex.printStackTrace();
        }
    }
}
