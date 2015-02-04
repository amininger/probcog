package probcog.classify;

import java.util.*;

import april.util.*;

import probcog.lcmtypes.*;

/** Stores tag detection information regarding labels, but with some persistence
 *  information. This allows us to make a one-time assignment of a label to a tag
 *  the first time we see it and then remember for subsequent observations of the
 *  tag in the near-term what it that label should be. This should *also* be
 *  useful in remembering that we've seen a tag before when beginning a new
 *  control behavior.
 **/
public class TagHistory
{
    static final long MAX_AGE_MS = 500;

    HashMap<Integer, Record> history;
    private class Record
    {
        public long utime = 0;         // Last utime of a recorded message
        public String label;       // Label we committed to for our observation
    }


    public TagHistory()
    {
        history = new HashMap<Integer, Record>();
    }

    public boolean addObservation(classification_t classy)
    {
        return addObservation(classy, classy.utime);
    }

    /** Insert a classification into the history. Returns TRUE if this is
     *  the first observation ever/since expiry, else false. Either way, updates
     *  timestamp of last observation from classy.
     */
    public boolean addObservation(classification_t classy, long utime)
    {
        if (!history.containsKey(classy.id))
            history.put(classy.id, new Record());

        Record r = history.get(classy.id);
        long diff = utime - r.utime;
        r.utime = utime;
        if (diff > MAX_AGE_MS*1000) {
            r.label = classy.name;
            return true;
        }

        return false;
    }

    public String getLabel(int id)
    {
        return getLabel(id, TimeUtil.utime());
    }

    /** Get the label of a given tag at time utime. If any previous observations
     *  have been made, records the appropriate label. Otherwise, returns an
     *  empty string, which we take to be the same as the object not having any
     *  label.
     **/
    public String getLabel(int id, long utime)
    {
        Record r = history.get(id);
        if (r == null)
            return "";
        if (utime > r.utime + MAX_AGE_MS*1000)
            return "";
        return r.label;
    }
}
