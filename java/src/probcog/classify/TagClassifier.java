package probcog.classify;

import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.lcmtypes.*;

public class TagClassifier
{
    static Random classifierRandom = new Random(8437531);

    HashMap<Integer, ArrayList<TagClass>> idToTag;

    LCM lcm = LCM.getSingleton();


    public TagClassifier(Config config)
    {
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
    }

    private void publishDetections(tag_detection_t[] tagList)
    {
        ArrayList<classification_t> classies = new ArrayList<classification_t>();

        for(tag_detection_t td : tagList) {
            if(idToTag.containsKey(td.id)) {
                // Make a classification_t for each thing the tag can be
                ArrayList<TagClass> classes = idToTag.get(td.id);
                for(TagClass tc : classes) {
                    classification_t classy = new classification_t();
                    classy.name = tc.label;
                    classy.id = td.id;
                    classy.xyzrpy = new double[6]; // XXX - WRONG! FIX!
                    classy.confidence = sampleConfidence(tc.mean,
                                                         tc.stddev);
                    classies.add(classy);
                }
            }
        }

        classification_list_t classy_list = new classification_list_t();
        classy_list.utime = TimeUtil.utime();
        classy_list.num_classifications = classies.size();
        classy_list.classifications = classies.toArray(new classification_t[classies.size()]);

        lcm.publish("CLASSIFICATIONS", classy_list);
    }

    private double sampleConfidence(double u, double s)
    {
        return MathUtil.clamp(u + classifierRandom.nextGaussian()*s, 0, 1);
    }


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
}

