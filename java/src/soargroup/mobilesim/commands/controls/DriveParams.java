package soargroup.mobilesim.commands.controls;

import soargroup.mobilesim.lcmtypes.*;
import magic2.lcmtypes.*;

/** A container class holding possible inputs to a control law.
 *  It is expected that these will be a fairly small subset of
 *  things, but this may eventually grow bloated, at which point
 *  reevaluation should be considered.
 **/
public class DriveParams
{
    public tag_classification_t classy; // XXX
    public grid_map_t gm;
    public laser_t laser;
    public pose_t pose;
    public double dt;
    public double heading;          // XXX

    public PotentialUtil.Params pp = null;

    public DriveParams()
    {

    }
}
