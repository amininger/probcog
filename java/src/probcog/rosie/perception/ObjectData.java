package probcog.rosie.perception;

import java.util.ArrayList;

import javax.json.*;

import april.jmat.*;

import probcog.perception.*;
import probcog.util.*;

public class ObjectData {

    private int id;
    private ArrayList<CategorizedData> catDat;

    private double[] pos;

    private ArrayList<String> stateValues;

    private double[] bboxDim;
    private double[] bboxPos;

    public ObjectData()
    {
        pos = new double[6];

        bboxPos = new double[6];
    	bboxDim = new double[3];

        catDat = new ArrayList<CategorizedData>();
    	stateValues = new ArrayList<String>();
    }

    public ObjectData(Obj o)
    {
        id = o.getID();

        pos = o.getPose();

        BoundingBox bbox = o.getBoundingBox();
        bboxPos = bbox.xyzrpy;
    	bboxDim = bbox.lenxyz;

        catDat = o.getCategoryData();
    	stateValues = o.getStates();
    }

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

        pos = new double[6];
        JsonObject xform = null;

        try {
            xform = msg.getJsonObject("pos");
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData pos");
        }

        try {
            JsonObject xyz = xform.getJsonObject("translation");
            pos[0] = xyz.getJsonNumber("x").doubleValue();
            pos[1] = xyz.getJsonNumber("y").doubleValue();
            pos[2] = xyz.getJsonNumber("z").doubleValue();
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData pose translation");
        }

        double[] quat = new double[4];
        try{
            JsonObject q = xform.getJsonObject("rotation");
            quat[0] = q.getJsonNumber("x").doubleValue();
            quat[1] = q.getJsonNumber("y").doubleValue();
            quat[2] = q.getJsonNumber("z").doubleValue();
            quat[3] = q.getJsonNumber("w").doubleValue();
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData pose rotation");
        }

        try {
            double[] rpy = LinAlg.quatToRollPitchYaw(quat);
            for (int i = 3; i < 6; i++) pos[i] = rpy[i-3];
        } catch (Exception e) {
            System.out.println("ERROR: Converting quat to rpy.");
        }

        bboxDim = new double[3];
        try {
            JsonObject dim = msg.getJsonObject("bbox_dim");
            bboxDim[0] = dim.getJsonNumber("x").doubleValue();
            bboxDim[1] = dim.getJsonNumber("y").doubleValue();
            bboxDim[2] = dim.getJsonNumber("z").doubleValue();
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData bbox_dim");
        }

        bboxPos = new double[6];

        try {
            xform = msg.getJsonObject("bbox_xyzrpy");
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData bbox_xyzrpy");
        }

        try {
            JsonObject xyz = xform.getJsonObject("translation");
            bboxPos[0] = xyz.getJsonNumber("x").doubleValue();
            bboxPos[1] = xyz.getJsonNumber("y").doubleValue();
            bboxPos[2] = xyz.getJsonNumber("z").doubleValue();
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData pose translation");
        }

        quat = new double[4];
        try{
            JsonObject q = xform.getJsonObject("rotation");
            quat[0] = q.getJsonNumber("x").doubleValue();
            quat[1] = q.getJsonNumber("y").doubleValue();
            quat[2] = q.getJsonNumber("z").doubleValue();
            quat[3] = q.getJsonNumber("w").doubleValue();
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData pose rotation");
        }

        try {
            double[] rpy = LinAlg.quatToRollPitchYaw(quat);
            for (int i = 3; i < 6; i++) bboxPos[i] = rpy[i-3];
        } catch (Exception e) {
            System.out.println("ERROR: Converting quat to rpy.");
        }

        int numStates = -1;
        try {
            numStates = msg.getInt("num_states");
        } catch (Exception e) {
            System.out.println("ERROR: Reading num_states");
        }

        try {
            JsonArray ja = msg.getJsonArray("state_values");
            for (int i = 0; i < ja.size(); i++) {
                stateValues.add(ja.getString(i));
            }
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData state values");
        }

        try {
            JsonArray ja = msg.getJsonArray("cat_dat");
            for (int i = 0; i < ja.size(); i++) {
                catDat.add(new CategorizedData(ja.getJsonObject(i)));
            }
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData categorized data");
        }
    }

    public JsonObject toJson()
    {
        double[] tmpRot = new double[]{pos[3], pos[4], pos[5]};
        double[] q = LinAlg.rollPitchYawToQuat(tmpRot);

        double[] tmpRotB = new double[]{bboxPos[3], bboxPos[4], bboxPos[5]};
        double[] qB = LinAlg.rollPitchYawToQuat(tmpRotB);

        JsonArrayBuilder jab = Json.createArrayBuilder();
        for (String s : stateValues) {
            jab = jab.add(s);
        }

        JsonArrayBuilder catjab = Json.createArrayBuilder();
        for (CategorizedData cd : catDat) {
            JsonArrayBuilder labjab = Json.createArrayBuilder();
            for (String s : cd.getLabel()) {
                labjab = labjab.add(s);
            }

            JsonArrayBuilder confjab = Json.createArrayBuilder();
            for (Double d : cd.getConfidence()) {
                confjab = confjab.add(d);
            }

            JsonArrayBuilder featjab = Json.createArrayBuilder();
            for (Double d : cd.getFeatures()) {
                featjab = featjab.add(d);
            }

            catjab = catjab.add(Json.createObjectBuilder()
                                .add("cat_type", cd.getCatType().ordinal())
                                .add("len", cd.numLabels())
                                .add("num_features", cd.numFeatures())
                                .add("label", labjab)
                                .add("confidence", confjab)
                                .add("features", featjab));
        }

        JsonObject jo = Json.createObjectBuilder()
            .add("obj_id", id)
            .add("pos", Json.createObjectBuilder()
                 .add("translation", Json.createObjectBuilder()
                      .add("x", bboxPos[0])
                      .add("y", bboxPos[1])
                      .add("z", bboxPos[2]))
                 .add("rotation", Json.createObjectBuilder()
                      .add("x", qB[0])
                      .add("y", qB[1])
                      .add("z", qB[2])
                      .add("w", qB[3])))
            .add("bbox_dim", Json.createObjectBuilder()
                 .add("x", bboxDim[0])
                 .add("y", bboxDim[1])
                 .add("z", bboxDim[2]))
            .add("bbox_xyzrpy", Json.createObjectBuilder()
                 .add("translation", Json.createObjectBuilder()
                      .add("x", bboxPos[0])
                      .add("y", bboxPos[1])
                      .add("z", bboxPos[2]))
                 .add("rotation", Json.createObjectBuilder()
                      .add("x", qB[0])
                      .add("y", qB[1])
                      .add("z", qB[2])
                      .add("w", qB[3])))
            .add("num_states", stateValues.size())
            .add("state_values", jab)
            .add("num_cat", numCat())
            .add("cat_dat", catjab)
            .build();
        return jo;
    }

    public int getID()
    {
        return id;
    }

    public void setID(int nid)
    {
        id = nid;
    }

    public int numCat()
    {
        return catDat.size();
    }

    public int numStates()
    {
        return stateValues.size();
    }

    public ArrayList<String> getStateValues()
    {
        return stateValues;
    }

    public ArrayList<CategorizedData> getCatDat()
    {
        return catDat;
    }

    public CategorizedData getCatDat(int i)
    {
        return catDat.get(i);
    }

    public void setCatDat(ArrayList<CategorizedData> na)
    {
        catDat = na;
    }

    public void setPos(double[] npos)
    {
        if (npos.length != 6) {
            System.out.println("ERROR: Setting invalid object pos for ObjectData.");
        }
        pos = npos;
    }

    public double[] getPos()
    {
        return pos;
    }

    public double getPos(int i)
    {
        return pos[i];
    }

    public void setBBoxPos(double[] npos)
    {
        if (npos.length != 6) {
            System.out.println("ERROR: Setting invalid bbox pos for ObjectData.");
        }
        bboxPos = npos;
    }

    public double[] getBBoxPos()
    {
        return bboxPos;
    }

    public double getBBoxPos(int i)
    {
        return bboxPos[i];
    }

    public void setBBoxDim(double[] ndim)
    {
        if (ndim.length != 3) {
            System.out.println("ERROR: Setting invalid object pos for ObjectData.");
        }
        bboxDim = ndim;
    }

    public double[] getBBoxDim()
    {
        return bboxDim;
    }

    public double getBBoxDim(int i)
    {
        return bboxDim[i];
    }

    public void setPos(int i, double val)
    {
        pos[i] = val;
    }

    public void setBBoxPos(int i, double val)
    {
        bboxPos[i] = val;
    }

    public void setBBoxDim(int i, double val)
    {
        bboxDim[i] = val;
    }
}
