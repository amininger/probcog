package soargroup.mobilesim.util;

import april.jmat.*;

import april.lcmtypes.*;
import magic2.lcmtypes.*;

public class LCMUtil
{
    static public magic2.lcmtypes.pose_t a2mPose(april.lcmtypes.pose_t ap)
    {
        magic2.lcmtypes.pose_t mp = new magic2.lcmtypes.pose_t();
        mp.utime = ap.utime;
        mp.pos = LinAlg.copy(ap.pos);
        mp.vel = LinAlg.copy(ap.vel);
        mp.orientation = LinAlg.copy(ap.orientation);
        mp.rotation_rate = LinAlg.copy(ap.rotation_rate);
        mp.accel = LinAlg.copy(ap.accel);

        return mp;
    }
}
