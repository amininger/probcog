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

public class TagClassifier implements LCMSubscriber
{
    static final double tagSize_m = Util.getConfig().requireDouble("tag_detection.tag.size_m");

    static Random classifierRandom = new Random(8437531);

    HashMap<Integer, ArrayList<TagClass>> idToTag;

    LCM lcm = LCM.getSingleton();

    /**
     * Read the config file and store information about each tag. Each
     * tag can have multiple "TagClasses" associated, and each tag class
     * contains a label name and a mean and standard deviation that
     * define the probability it belongs to the labeled class. Multiple
     * tags can belong to the same class. If an april tag has mulitple
     * tag classes associated with it they will all be reported when we
     * see it and publish classifications.
     **/
    public TagClassifier(GetOpt opts) throws IOException
    {
        Config config = new ConfigFile(opts.getString("config"));

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

        lcm.subscribe("TAG_DETECTIONS", this);
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

            // Make a classification_t for each thing the tag can be
            ArrayList<TagClass> classes = idToTag.get(td.id);
            for(TagClass tc : classes) {

                classification_t classy = new classification_t();
                classy.name = tc.label;
                classy.id = td.id;
                classy.xyzrpy = LinAlg.matrixToXyzrpy(T2B);
                classy.confidence = sampleConfidence(tc.mean, tc.stddev);
                classies.add(classy);
            }
        }

        classification_list_t classy_list = new classification_list_t();
        classy_list.utime = TimeUtil.utime();
        classy_list.num_classifications = classies.size();
        classy_list.classifications = classies.toArray(new classification_t[classies.size()]);

        lcm.publish("CLASSIFICATIONS", classy_list);
    }


    /**
     * Given the mean and standard deviation, get a Gaussian probability.
     **/
    private double sampleConfidence(double u, double s)
    {
        return MathUtil.clamp(u + classifierRandom.nextGaussian()*s, 0, 1);
    }


    /**
     * Listen to lcm messages.
     **/
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }


    /**
     * In particular listen for tag detection messages. When one of these
     * is seen, process the tags by determining what classes/labels they
     * stand for.
     **/
    private void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("TAG_DETECTIONS")) {
            pan_tilt_tag_detections_t tagList = new pan_tilt_tag_detections_t(ins);
        }
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
        GetOpt opts = new GetOpt();

        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('c', "config", null, "Config file that denotes april tag meanings");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }
        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }

        try {
           TagClassifier tc = new TagClassifier(opts);
         } catch (IOException ioex) {
            System.err.println("ERR: Error starting tag classifier");
            ioex.printStackTrace();
            System.exit(1);
        }
    }
}

