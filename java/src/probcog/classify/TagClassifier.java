package probcog.classify;

import java.util.*;

import april.config.*;

public class TagClassifier
{
    HashMap<Integer, Tag> idToTag;

    public TagClassify(Config config)
    {
        idToTag = new HashMap<Integer, Tag>();

        // Read in all possible tag classes with associated tag ids
        // and store them in hashmap accessed by tag id.
        for (int i = 0;; i++) {
            String label = config.getString("classes.c"+i+".label", "");
            int[] ids =  = config.getInts("classes.c"+i+".ids", null);
            double prob =  = config.getDouble("classes.c"+i+".probability", null);
            if(label.equals("") || (ids == null))
                break;

            TagClass tc = new TagClass(label, prob);
            for(int id : ids) {
                Tag tag;
                if(idToTag.containsKey(id))
                    tag = idToTag.get(id);
                else
                    tag = new Tag(id);
                tag.addClass(tc);
                idToTag.put(id, tag);
            }
        }
    }

    class Tag
    {
        int tagID;
        ArrayList<TagClass> classes;

        public Tag(int tagID)
        {
            this.tagID = tagID;
            classes = new ArrayList<TagClass>();
        }
        public Tag(int tagID, ArrayList<TagClass> classes)
        {
            this.tagID = tagID;
            this.classes = classes;
        }
        public Tag(int tagID, TagClass _class)
        {
            this.tagID = tagID;
            classes = new ArrayList<TagClass>();
            classes.add(_class);
        }

        public void addClass(TagClass tc)
        {
            classes.add(tc);
        }
    }

    class TagClass
    {
        String label;
        double prob;

        public TagClass(String label, double prob)
        {
            this.label = label;
            this.prob = prob;
        }
    }
}

