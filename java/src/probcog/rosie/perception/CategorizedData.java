package probcog.rosie.perception;

import java.util.ArrayList;

import javax.json.*;

public class CategorizedData {
    public enum CategoryType {
        CAT_COLOR, CAT_SHAPE, CAT_SIZE, CAT_LOCATION, CAT_WEIGHT, CAT_TEMPERATURE
    }

    private CategoryType catType;
    private ArrayList<String> label;
    private ArrayList<Double> confidence;
    private ArrayList<Double> features;

    public CategorizedData(JsonObject msg)
    {
        catType = CategoryType.values()[msg.getInt("cat_type")];

        JsonArray l = msg.getJsonArray("label");
        label = new ArrayList<String>();
        for (int i = 0; i < l.size(); i++) {
            label.add(l.getString(i));
        }

        JsonArray c = msg.getJsonArray("confidence");
        confidence = new ArrayList<Double>();
        for (int i = 0; i < c.size(); i++) {
            confidence.add(c.getJsonNumber(i).doubleValue());
        }

        JsonArray f = msg.getJsonArray("features");
        features = new ArrayList<Double>();
        for (int i = 0; i < f.size(); i++) {
            features.add(f.getJsonNumber(i).doubleValue());
        }
    }

    public CategoryType getCatType()
    {
        return catType;
    }

    public int numLabels()
    {
        return label.size();
    }

    public int numFeatures()
    {
        return features.size();
    }

    public ArrayList<String> getLabel()
    {
        return label;
    }

    public ArrayList<Double> getConfidence()
    {
        return confidence;
    }

    public ArrayList<Double> getFeatures()
    {
        return features;
    }
}
