package probcog.rosie.perception;

import java.util.ArrayList;

import javax.json.*;

import april.jmat.*;

public class ObjectData {

    private int id;
    private ArrayList<CategorizedData> catDat;

    private double[] pos;

    private ArrayList<String> stateValues;

    private double[] bboxDim;
    private double[] bboxPos;

    public ObjectData(JsonObject msg)
    {
        try {
            id = msg.getInt("obj_id");
        }
        catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData obj_id");
        }

        try {
            JsonArray cd = msg.getJsonArray("cat_dat");
            catDat = new ArrayList<CategorizedData>();
            for (int i = 0; i < cd.size(); i++) {
                catDat.add(new CategorizedData(cd.getJsonObject(i)));
            }
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData cat_dat");
        }

        try {
            JsonArray sv = msg.getJsonArray("state_values");
            stateValues = new ArrayList<String>();
            for (int i = 0; i < sv.size(); i++) {
                stateValues.add(sv.getString(i));
            }
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData state_values");
        }

        //x reading failing here
        pos = new double[6];
        try {
            JsonObject xform = msg.getJsonObject("pos");
            JsonObject xyz = xform.getJsonObject("translation");
            pos[0] = xyz.getJsonNumber("x").doubleValue();
            pos[1] = xyz.getJsonNumber("y").doubleValue();
            pos[2] = xyz.getJsonNumber("z").doubleValue();

            double[] quat = new double[4];
            JsonObject q = xform.getJsonObject("rotation");
            quat[0] = q.getJsonNumber("x").doubleValue();
            quat[1] = q.getJsonNumber("y").doubleValue();
            quat[2] = q.getJsonNumber("z").doubleValue();
            quat[3] = q.getJsonNumber("w").doubleValue();

            double[] rpy = LinAlg.quatToRollPitchYaw(quat);
            for (int i = 3; i < 6; i++) pos[i] = rpy[i];
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData pos");
        }

        bboxDim = new double[3];
        try {
            JsonObject dim = msg.getJsonObject("bbox_dim");
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData bbox_dim");
        }
        /// ETC
    }

    public int getID()
    {
        return id;
    }

    public int numCat()
    {
        return catDat.size();
    }

    public int numStates()
    {
        return stateValues.size();
    }

    public ArrayList<CategorizedData> getCatDat()
    {
        return catDat;
    }
}
