package probcog.rosie.perception;

import java.util.ArrayList;

import javax.json.*;

import april.util.TimeUtil;

import probcog.perception.*;
import probcog.util.*;

public class TrainingLabel {
    private int id;
    private String label;
    private CategorizedData.CategoryType cat;
    private long utime;

    public TrainingLabel(int i, String l, CategorizedData.CategoryType c)
    {
        id = i;
        label = l;
        cat = c;
        utime = TimeUtil.utime();
    }

    public TrainingLabel()
    {
        id = 0;
        label = "";
        utime = TimeUtil.utime();
    }

    public long getTime()
    {
        return utime;
    }

    public JsonObject toJson()
    {
        JsonObject jo = Json.createObjectBuilder()
            .add("id", id)
            .add("label", label)
            .add("utime", utime)
            .add("cat", cat.ordinal()).build();
        return jo;
    }

    public int getId()
    {
        return id;
    }

    public String getLabel()
    {
        return label;
    }

    public CategorizedData.CategoryType getCatType()
    {
        return cat;
    }
}
