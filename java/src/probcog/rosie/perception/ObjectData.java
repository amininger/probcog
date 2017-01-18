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

    	//objData.num_cat = 0;
    	//objData.cat_dat = new categorized_data_t[0];

    	stateValues = new ArrayList<String>();
    }

    public ObjectData(Obj o)
    {
        pos = o.getPose();

        bboxPos = o.getBoundingBox().xyzrpy;
    	bboxDim = o.getBoundingBox().lenxyz;

    	//objData.num_cat = 0;
    	//objData.cat_dat = new categorized_data_t[0];

    	stateValues = new ArrayList<String>();
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

        /// ETC
    }

    public String jsonVal(String name, String object)
    {
        return ("\"" + name + "\": " + object);
    }

    public String jsonVal(String name, double object)
    {
        return ("\"" + name + "\": " + object);
    }

    public String jsonVal(String name, int object)
    {
        return ("\"" + name + "\": " + object);
    }

    public String jsonItem(String name, String object)
    {
        return ("\"" + name + "\": {" + object + "}");
    }

    public String toJsonString()
    {
        StringBuilder curObj = new StringBuilder();
        curObj.append("{");
        curObj.append(jsonVal("obj_id", id) + ", ");
        double[] tmpRot = new double[]{pos[3], pos[4], pos[5]};
        double[] q = LinAlg.rollPitchYawToQuat(tmpRot);

        curObj.append(jsonItem("pos",
                               jsonItem("translation",
                                        jsonVal("x", pos[0]) + ", " +
                                        jsonVal("y", pos[1]) + ", " +
                                        jsonVal("z", pos[2])) +
                               ", " +
                               jsonItem("rotation",
                                        jsonVal("x", q[0]) + ", " +
                                        jsonVal("y", q[1]) + ", " +
                                        jsonVal("z", q[2]) + ", " +
                                        jsonVal("w", q[3]))));
        curObj.append(",");

        curObj.append(jsonItem("bbox_dim",
                               jsonVal("x", bboxDim[0]) + ", " +
                               jsonVal("y", bboxDim[1]) + ", " +
                               jsonVal("z", bboxDim[2])));
        curObj.append(",");

        double[] tmpRotB = new double[]{bboxPos[3], bboxPos[4], bboxPos[5]};
        double[] qB = LinAlg.rollPitchYawToQuat(tmpRotB);

        curObj.append(jsonItem("pos",
                               jsonItem("translation",
                                        jsonVal("x", bboxPos[0]) + ", " +
                                        jsonVal("y", bboxPos[1]) + ", " +
                                        jsonVal("z", bboxPos[2])) +
                               ", " +
                               jsonItem("rotation",
                                        jsonVal("x", qB[0]) + ", " +
                                        jsonVal("y", qB[1]) + ", " +
                                        jsonVal("z", qB[2]) + ", " +
                                        jsonVal("w", qB[3]))));
        curObj.append(",");

        curObj.append(jsonVal("num_states", stateValues.size()));
        curObj.append(",");

        curObj.append("\"state_values\": [");
        int count = 0;
        for (String s : stateValues) {
            if (count > 0) curObj.append(", ");
            count++;
            curObj.append(s);
        }
        curObj.append("]");
        //curObj.append(",");

        // categorized_data_t[] cat_dat = ob.getCategoryData();
        // od.num_cat = cat_dat.length;
        // od.cat_dat = cat_dat;

        curObj.append("}");
        return curObj.toString();
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

    public ArrayList<CategorizedData> getCatDat()
    {
        return catDat;
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
