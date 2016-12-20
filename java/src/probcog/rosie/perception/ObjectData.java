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

    public ObjectData()
    {
        pos = new double[6];

        bboxPos = new double[6];
    	bboxDim = new double[3];

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
        } catch (Exception e) {
            System.out.println("ERROR: Reading ObjectData bbox_dim");
        }
        /// ETC
    }

    // TODO unify with Tracker?
    public String toJsonString()
    {
        StringBuilder curObj = new StringBuilder();
        curObj.append("{\"obj_id\": " + id + ", ");
        curObj.append("\"pos\": {\"translation\": {");
        curObj.append("\"x\": " + pos[0] + ", ");
        curObj.append("\"y\": " + pos[1] + ", ");
        curObj.append("\"z\": " + pos[2] + "}, ");

        double[] tmpRot = new double[]{pos[3], pos[4], pos[5]};
        double[] q = LinAlg.rollPitchYawToQuat(tmpRot);
        curObj.append("\"rotation\": {");
        curObj.append("\"x\": " + q[0] + ", ");
        curObj.append("\"y\": " + q[1] + ", ");
        curObj.append("\"z\": " + q[2] + ", ");
        curObj.append("\"w\": " + q[3] + "}}, ");

        // BoundingBox bbox = ob.getBoundingBox();
        // od.bbox_dim = bbox.lenxyz;
        // od.bbox_xyzrpy = bbox.xyzrpy;

        // od.state_values = ob.getStates();
        // od.num_states = od.state_values.length;

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

    public void setBBoxPos(double[] npos)
    {
        if (npos.length != 6) {
            System.out.println("ERROR: Setting invalid bbox pos for ObjectData.");
        }
        bboxPos = npos;
    }

    public void setBBoxDim(double[] ndim)
    {
        if (ndim.length != 3) {
            System.out.println("ERROR: Setting invalid object pos for ObjectData.");
        }
        bboxDim = ndim;
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
