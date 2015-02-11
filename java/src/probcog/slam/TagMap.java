package probcog.slam;

import java.io.*;
import java.util.*;

import april.jmat.*;
import april.util.*;

import magic2.lcmtypes.*;   // XXX

/** Loads a map with registered tag positions/orientations from file. Lets
 *  the user do convenient things like query for a 2D laser scan for a given
 *  XYT or query about tag poses in the world relative to a particular
 *  XYT.
 **/
public class TagMap
{
    public GridMap gm = GridMap.makeMeters(0,0,1,1,0.05,0);
    public ArrayList<TagXYT> tags = new ArrayList<TagXYT>();

    public static class TagXYT
    {
        public int id;
        public double[] xyt;

        public TagXYT(int id, double[] xyt)
        {
            this.id = id;
            this.xyt = xyt;
        }
    }

    private TagMap(GridMap gm, ArrayList<TagXYT> tags)
    {
        this.gm = gm;
        this.tags = tags;
    }

    static TagMap load(StructureReader fin) throws IOException
    {
        double x0 = fin.readDouble();
        double y0 = fin.readDouble();
        int w = fin.readInt();
        int h = fin.readInt();
        double mpp = fin.readDouble();
        byte[] data = fin.readBytes();

        GridMap gm = GridMap.makePixels(x0, y0, w, h, mpp, 0, data);

        ArrayList<TagXYT> tags = new ArrayList<TagXYT>();
        int n = fin.readInt();
        for (int i = 0; i < n; i++) {
            fin.blockBegin();
            int id = fin.readInt();
            double[] xyt = fin.readDoubles();
            fin.blockEnd();

            tags.add(new TagXYT(id, xyt));
        }

        System.out.printf("Loaded map [%f %f %d %d %f] w/ %d tags\n",
                          x0, y0, w, h, mpp, tags.size());

        return new TagMap(gm, tags);
    }

    // === Map queries for simulation =====================================
    public laser_t getLaser(double[] xyt)
    {
        return getLaser(xyt, 135.0, Math.toRadians(3), 5.0);
    }

    /** Get a laser scan for the given position.
     *  @param xyt          Robot XYT
     *  @param thetaRange   The range out from 0 (+/-) that we will scan
     *  @param radstep      The step size between range measurements in rads
     *  @param maxRange     The maximum range measurement we'll take.
     *
     *  @return A laser_t with -1 indicating bad measurements. No utime
     **/
    public laser_t getLaser(double[] xyt, double thetaRange, double radstep, double maxRange)
    {
        double minTheta = -Math.toRadians(Math.abs(thetaRange));
        double maxTheta = Math.toRadians(Math.abs(thetaRange));
        double rs = Math.abs(radstep);

        laser_t laser = new laser_t();
        laser.rad0 = (float)minTheta;
        laser.radstep = (float)rs;
        laser.nranges = (int)((maxTheta-minTheta)/rs);
        laser.ranges = new float[laser.nranges];

        double[] xy = new double[2];
        for (int i = 0; i < laser.nranges; i++)
        {
            double theta = MathUtil.mod2pi(minTheta+xyt[2]+i*rs);
            laser.ranges[i] = -1;
            for (double r = 0; r <= maxRange; r+= gm.metersPerPixel/2) {
                double x = xyt[0] + r*Math.cos(theta);
                double y = xyt[1] + r*Math.sin(theta);
                if (gm.getValue(x, y) != 0) {
                    laser.ranges[i] = (float)r;
                    break;
                }
            }
        }

        return laser;
    }

    public ArrayList<TagXYT> getTags(double[] xyt)
    {
        return getTags(xyt, 2.0);
    }

    /** Get the tags visible from a particular location and orientation.
     *  @param xyt              Robot XYT
     *  @param detectionRange   Maximum range tags are visible from
     *
     *  @return A list of tags w/relative locations and orientations to robot
     **/
    public ArrayList<TagXYT> getTags(double[] xyt, double detectionRange)
    {
        ArrayList<TagXYT> seen = new ArrayList<TagXYT>();
        for (int i = 0; i < tags.size(); i++) {
            double dist = LinAlg.distance(xyt, tags.get(i).xyt, 2);
            if (dist > detectionRange)
                continue;

            int id = tags.get(i).id;
            double[] txyt = tags.get(i).xyt;
            seen.add(new TagXYT(id, LinAlg.xytInvMul31(xyt, txyt)));
        }

        return seen;
    }

    /** Return a tag position given its ID
     *  @param id   Tag ID
     *
     *  @return The corresponding TagXYT. null if no such tag exists
     **/
    public TagXYT getTag(int id)
    {
        for (TagXYT tag: tags) {
            if (tag.id == id)
                return tag;
        }

        return null;
    }

    /** Return an estimated robot pose based on its current
     *  tag observations.
     *  @param relativeObservations Relative tag XYT to robot
     *
     *  @return A robot XYT based on our observations.
     **/
    public double[] getXYTFromTags(ArrayList<TagXYT> relativeObservations)
    {
        assert (relativeObservations != null);
        assert (relativeObservations.size() > 0);

        // Extract pose from tags.
        return null;
    }
}
