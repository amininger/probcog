package probcog.classify;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;
import april.tag.TagDetection;

import probcog.lcmtypes.*;
import probcog.util.*;

public class TagClassifier
{
    String tagConfig = Util.getConfig().requireString("tag_config");
    double tagSize_m = Util.getConfig().requireDouble("tag_detection.tag.size_m");

    //static Random classifierRandom = new Random(104395301);
    Random classifierRandom = new Random();
    LCM lcm = LCM.getSingleton();

    HashMap<Integer, ArrayList<TagClass>> idToTag;
    HashMap<String, Set<Integer> > tagClassToIDs;
    HashSet<String> tagClasses = new HashSet<String>();

    public TagClassifier() throws IOException
    {
        this(true);
    }

    /**
     * Read the config file and store information about each tag. Each
     * tag can have multiple "TagClasses" associated, and each tag class
     * contains a label name and a mean and standard deviation that
     * define the probability it belongs to the labeled class. Multiple
     * tags can belong to the same class. If an april tag has mulitple
     * tag classes associated with it they will all be reported when we
     * see it and publish classifications.
     **/
    public TagClassifier(boolean useLcm) throws IOException
    {
        Config config = new ConfigFile(tagConfig);

        idToTag = new HashMap<Integer, ArrayList<TagClass>>();
        tagClassToIDs = new HashMap<String, Set<Integer> >();

        // Read in all possible tag classes with associated tag ids
        // and store them in hashmap accessed by tag id.
        for (int i = 0;; i++) {

            String[] labels = config.getStrings("classes.c"+i+".labels", null);
            double[] probs = config.getDoubles("classes.c"+i+".probs", null);
            int[] ids = config.getInts("classes.c"+i+".ids", null);
            double mean = config.getDouble("classes.c"+i+".mean", 0);
            double stddev = config.getDouble("classes.c"+i+".stddev", 0);
            double minRange = config.getDouble("classes.c"+i+".minRange", 0);
            double maxRange = config.getDouble("classes.c"+i+".maxRange", 0);

            if (labels == null)
                break;

            if (!tagClassToIDs.containsKey(labels[0]))
                tagClassToIDs.put(labels[0], new HashSet<Integer>());

            for (String label: labels) {
                if (label.equals(""))
                    continue;
                tagClasses.add(label);
            }

            ArrayList<String> labelList = new ArrayList<String>();
            for (String label: labels)
                labelList.add(label);
            ArrayList<Double> probList = new ArrayList<Double>();
            for (double prob: probs)
                probList.add(prob);

            TagClass tag = new TagClass(labelList, probList, mean, stddev, minRange, maxRange);
            for(int id : ids) {
                ArrayList<TagClass> allTags;
                if(idToTag.containsKey(id))
                    allTags = idToTag.get(id);
                else
                    allTags = new ArrayList<TagClass>();

                allTags.add(tag);
                idToTag.put(id, allTags);
                tagClassToIDs.get(labels[0]).add(id);
            }
        }

        if (useLcm)
            new ListenerThread().start();
    }

    /** Get an exhaustive list of all of the tag IDs matching a particular class */
    public Set<Integer> getIDsForClass(String tagClass)
    {
        if (tagClassToIDs.containsKey(tagClass))
            return tagClassToIDs.get(tagClass);
        return new HashSet<Integer>();
    }

    /** Get an exhaustive list of the classes of tags that exist in the world. */
    public Set<String> getAllClasses()
    {
        return tagClasses;  // Not data safe. READ ONLY USE PLEASE
    }

    /** Get the classes associated with a particular tag */
    public Set<String> getClasses(int id)
    {
        HashSet<String> classes = new HashSet<String>();
        ArrayList<TagClass> tcs = idToTag.get(id);
        if (tcs == null)
            return classes;

        for (TagClass tc: tcs)
            classes.addAll(tc.labels);

        return classes;
    }

    /** Expose the probability that a tag is CORRECTLY labeled. Assumes you
     *  only care about the first class (since we only ever have one class
     *  right now.
     **/
    public double correctProbability(int tagID)
    {
        if (!idToTag.containsKey(tagID))
            return 0;

        ArrayList<TagClass> tcs = idToTag.get(tagID);
        if (tcs.size() < 1)
            return 0;

        TagClass tc = tcs.get(0);
        return tc.probs.get(0);
    }

    public String correctClass(int tagID)
    {
        if (!idToTag.containsKey(tagID))
            return  "";

        ArrayList<TagClass> tcs = idToTag.get(tagID);
        if (tcs.size() < 1)
            return "";

        TagClass tc = tcs.get(0);
        return tc.labels.get(0);
    }

    public double sampleRange(int tagID)
    {
        if (!idToTag.containsKey(tagID))
            return -1;

        ArrayList<TagClass> tcs = idToTag.get(tagID);
        if (tcs.size() < 1)
            return -1;

        TagClass tc = tcs.get(0);

        double v = MathUtil.clamp(classifierRandom.nextGaussian(), -3, 3);
        double diff = tc.maxRange - tc.minRange;
        double range = (tc.minRange+diff/2) + v*(diff/6);
        return range;
    }

    /** Get the max range we are capable of sensing this tag */
    public double getMaxRange(int tagID)
    {
        if (!idToTag.containsKey(tagID))
            return -1;

        ArrayList<TagClass> tcs = idToTag.get(tagID);
        if (tcs.size() < 1)
            return -1;

        TagClass tc = tcs.get(0);
        return tc.maxRange;
    }

    public ArrayList<classification_t> classifyTag(int id, double[] xyzrpy)
    {
        return classifyTag(id, xyzrpy, false);
    }

    /** Return a list of classifications for a tag of a given ID and xyzrpy
     *  relative to the robot
     **/
    public ArrayList<classification_t> classifyTag(int id, double[] xyzrpy, boolean perfect)
    {
        ArrayList<classification_t> classies = new ArrayList<classification_t>();
        ArrayList<TagClass> tcs = idToTag.get(id);
        if (tcs == null)
            return classies;    // No config entries

        for (TagClass tc: tcs) {
            double v = classifierRandom.nextDouble();
            if (perfect)
                v = 0.0;

            double sum = 0.0;
            String label = tc.labels.get(0);
            for (int i = 0; i < tc.labels.size(); i++) {
                sum += tc.probs.get(i);
                if (sum >= v) {
                    label = tc.labels.get(i);
                    break;
                }
            }

            classification_t classy = new classification_t();
            classy.name = label;
            classy.range = sampleRange(id); // When perfect, what should this do?
            classy.id = id;
            classy.xyzrpy = xyzrpy;
            classy.confidence = sampleConfidence(tc.mean, tc.stddev);
            classies.add(classy);
        }

        return classies;
    }

    private void publishDetections(pan_tilt_tag_detections_t tagList)
    {
        ArrayList<classification_t> classies = new ArrayList<classification_t>();

        for(int i=0; i<tagList.ndetects; i++) {

            if(!idToTag.containsKey(tagList.detections[i].id))
                continue;

            TagDetection td = new TagDetection();
            td.id = tagList.detections[i].id;
            td.hammingDistance = tagList.detections[i].errors;
            td.homography = tagList.detections[i].homography;
            td.hxy = tagList.detections[i].hxy;

            double[][] T2B = TagUtil.getTagToPose(tagList.cam_to_pose, td, tagSize_m);
            classies.addAll(classifyTag(td.id, LinAlg.matrixToXyzrpy(T2B)));
        }

        classification_list_t classy_list = new classification_list_t();
        classy_list.utime = TimeUtil.utime();
        classy_list.num_classifications = classies.size();
        classy_list.classifications = classies.toArray(new classification_t[classies.size()]);

        lcm.publish("CLASSIFICATIONS", classy_list);
    }


    class ListenerThread extends Thread implements LCMSubscriber
    {
        public ListenerThread()
        {
            lcm.subscribe("TAG_DETECTIONS", this);
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/60);
            }
        }


        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                messageReceivedEx(lcm, channel, ins);
            } catch (IOException ex) {
                System.out.println("WRN: "+ex);
            }
        }


        private void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
        {
            if (channel.equals("TAG_DETECTIONS")) {
                pan_tilt_tag_detections_t tagList = new pan_tilt_tag_detections_t(ins);
                publishDetections(tagList);
            }
        }
    }


    /**
     * Given the mean and standard deviation, get a Gaussian probability.
     **/
    private double sampleConfidence(double u, double s)
    {
        return MathUtil.clamp(u + classifierRandom.nextGaussian()*s, 0, 1);
    }

    /**
     * Keep track of the information associated with each tag id. Tags
     * belong to classes (i.e. door, hallway, red) and have some
     * associated probability (this mimics our uncertainly of real-world
     * classification.
     **/
    class TagClass
    {
        ArrayList<String> labels;
        ArrayList<Double> probs;
        double mean;
        double stddev;
        double minRange;
        double maxRange;

        public TagClass(ArrayList<String> labels,
                        ArrayList<Double> probs,
                        double mean,
                        double stddev,
                        double minRange,
                        double maxRange)
        {
            // Assume that the FIRST label is the real one.
            this.labels = labels;
            this.probs = probs;
            this.mean = mean;
            this.stddev = stddev;
            this.minRange = minRange;
            this.maxRange = maxRange;
        }

        public String toString()
        {
            return String.format("%f %f", minRange, maxRange);
        }
    }

    public static void main(String[] args)
    {
        try {
           TagClassifier tc = new TagClassifier();
         } catch (IOException ioex) {
            System.err.println("ERR: Error starting tag classifier");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}

