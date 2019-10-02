package soargroup.mobilesim.robot.perception;

import java.util.*;

import april.jmat.*;
import april.util.*;

import soargroup.mobilesim.lcmtypes.*;

/** Stores tag detection information regarding labels, but with some persistence
 *  information. This allows us to make a one-time assignment of a label to a tag
 *  the first time we see it and then remember for subsequent observations of the
 *  tag in the near-term what it that label should be. This should *also* be
 *  useful in remembering that we've seen a tag before when beginning a new
 *  control behavior.
 *
 *  NOTE: On first observation of tag, also record the distance we should start
 *  reporting that we see the landmark.
 **/
public class TagHistory
{
    static final long MAX_AGE_MS = 500;

    HashMap<Integer, Record> history;
    private class Record
    {
        public long utime = 0;            // Last utime of a recorded message
        //public String label;            // Label we committed to for our observation
        //public double range;            // Range at which we should first see tag
        public ArrayList<String> labels = new ArrayList<String>();
        public ArrayList<Double> ranges = new ArrayList<Double>();
        public boolean observed = false;  // Have we ACTUALLY sighted the tag yet?
    }


    public TagHistory()
    {
        history = new HashMap<Integer, Record>();
    }

    public void addObservations(ArrayList<tag_classification_t> classies)
    {
        if (classies.size() < 1)
            return;
        addObservations(classies, classies.get(0).utime);
    }

    /** Insert a classification into the history. Returns TRUE if this is
     *  the first observation ever/since expiry, else false. Either way, updates
     *  timestamp of last observation from classy.
     */
    public void addObservations(ArrayList<tag_classification_t> classies, long utime)
    {
        HashMap<Integer, ArrayList<String> > labelMap = new HashMap<Integer, ArrayList<String> >();
        HashMap<Integer, ArrayList<Double> > rangeMap = new HashMap<Integer, ArrayList<Double> >();
        for (tag_classification_t classy: classies) {
            if (!labelMap.containsKey(classy.tag_id)) {
                labelMap.put(classy.tag_id, new ArrayList<String>());
                rangeMap.put(classy.tag_id, new ArrayList<Double>());
            }
            labelMap.get(classy.tag_id).add(classy.name);
            rangeMap.get(classy.tag_id).add(classy.range);
        }

        for (int id: labelMap.keySet()) {
            if (!history.containsKey(id))
                history.put(id, new Record());

            Record r = history.get(id);
            long diff = utime - r.utime;
            r.utime = utime;
            if (diff > MAX_AGE_MS*1000) {
                r.labels = labelMap.get(id);
                r.ranges = rangeMap.get(id);
                r.observed = false;
            }
        }
    }

    public ArrayList<tag_classification_t> getLabels(int id, double[] xyzrpy)
    {
        return getLabels(id, xyzrpy, TimeUtil.utime());
    }

    /** Get the label of a given tag at time utime. If any previous observations
     *  have been made, records the appropriate label. Otherwise, returns an
     *  empty string, which we take to be the same as the object not having any
     *  label.
     **/
    public ArrayList<tag_classification_t> getLabels(int id, double[] xyzrpy, long utime)
    {
        double range = LinAlg.magnitude(LinAlg.resize(xyzrpy, 2));

        ArrayList<tag_classification_t> classies = new ArrayList<tag_classification_t>();
        Record r = history.get(id);
        if (r == null)
            return classies;
        if (utime > r.utime + MAX_AGE_MS*1000)
            return classies;

        // Populate list
        for (int i = 0; i < r.labels.size(); i++) {
            if (range > r.ranges.get(i))
                continue;
            tag_classification_t classy = new tag_classification_t();
            classy.utime = utime;
            classy.tag_id = id;
            classy.name = r.labels.get(i);
            classy.range = range;       // XXX This really shouldn't exist...
            classy.xyzrpy = xyzrpy;
            classy.confidence = 1.0;    // XXX

            classies.add(classy);
        }

        return classies;
    }
}
