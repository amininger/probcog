package probcog.commands.controls;

import java.util.*;

import probcog.util.*;

import magic2.lcmtypes.*;

/** A utility class for generating and debugging potential functions */
public class PotentialUtil
{
    static public class Params
    {
        public laser_t laser;
        public pose_t pose;
        public double[] goalXYT;

        public Params(laser_t laser, pose_t pose, double[] goalXYT)
        {
            this.laser = laser;
            this.pose = pose;
            this.goalXYT = goalXYT;
        }

        // XXX Other parameters to affect search here. Set to sane default
        // values for normal goto XY operation.

        // Is this value set correctly in config? Reevaluate for new robot.
        public double robotRadius = Util.getConfig().requireDouble("robot.geometry.radius");
    }

    /** Given application specific parameters, generate a potential field
     *  locally centered around the robot.
     **/
    static public void getPotential(Params params)
    {

    }

    static public void main(String[] args)
    {
        // XXX TODO: Test infrastructure
    }
}
