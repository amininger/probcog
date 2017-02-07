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

    public String toJsonString()
    {
        StringBuilder json = new StringBuilder();
        json.append("{");
        json.append("\"id\": " + id + ", ");
        json.append("\"label\": \"" + label + "\", ");
        json.append("\"utime\": " + utime + ", ");
        json.append("\"cat\":" + cat.ordinal());
        json.append("}");
        return json.toString();
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
