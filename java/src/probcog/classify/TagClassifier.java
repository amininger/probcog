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

    static Random classifierRandom = new Random(8437531);
    LCM lcm = LCM.getSingleton();

    HashMap<Integer, ArrayList<TagClass>> idToTag;
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

        // Read in all possible tag classes with associated tag ids
        // and store them in hashmap accessed by tag id.
        for (int i = 0;; i++) {

            String label = config.getString("classes.c"+i+".label", "");
            int[] ids = config.getInts("classes.c"+i+".ids", null);
            double mean = config.getDouble("classes.c"+i+".mean", 0);
            double stddev = config.getDouble("classes.c"+i+".stddev", 0);

            if(label.equals("") || (ids == null))
                break;

            tagClasses.add(label);

            TagClass tag = new TagClass(label, mean, stddev);
            for(int id : ids) {
                ArrayList<TagClass> allTags;
                if(idToTag.containsKey(id))
                    allTags = idToTag.get(id);
                else
                    allTags = new ArrayList<TagClass>();

                allTags.add(tag);
                idToTag.put(id, allTags);
            }
        }

        if (useLcm)
            new ListenerThread().start();
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
            classes.add(tc.label);

        return classes;
    }

    /** Return a list of classifications for a tag of a given ID and xyzrpy
     *  relative to the robot
     **/
    public ArrayList<classification_t> classifyTag(int id, double[] xyzrpy)
    {
        ArrayList<classification_t> classies = new ArrayList<classification_t>();
        ArrayList<TagClass> tcs = idToTag.get(id);
        if (tcs == null)
            return classies;    // No config entries

        for (TagClass tc: tcs) {
            classification_t classy = new classification_t();
            classy.name = tc.label;
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
        String label;
        double mean;
        double stddev;

        public TagClass(String label, double mean, double stdeev)
        {
            this.label = label;
            this.mean = mean;
            this.stddev = stddev;
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

