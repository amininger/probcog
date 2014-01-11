package probcog.sim; // XXX - Should it go here? XXX Probably not

import java.awt.Color;
import java.io.IOException;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.classify.Classifications;
import probcog.classify.Features.FeatureCategory;
import probcog.perception.*;
import probcog.util.*;

public class SimLocation extends SimObjectPC
{
    private String name;
    protected double[] lwh = new double[]{1, 1, 1};

    public SimLocation(SimWorld sw)
    {
    	super(sw);
    }

    public VisObject getVisObject()
    {
        ArrayList<Object> objs = new ArrayList<Object>();
        
        objs.add(LinAlg.scale(scale));

        // The larger box making up the background of the object
        objs.add(new VisChain(LinAlg.translate(0, 0, 1), new VzRectangle(new VzMesh.Style(color))));

        // The smaller inner box is only drawn if there is a door and it's open
        if(currentState.containsKey("door") && currentState.get("door").equals("open")) {
            objs.add(new VisChain(LinAlg.translate(0,0,1.001),
                                  LinAlg.scale(.9),
                                  new VzRectangle(new VzMesh.Style(Color.DARK_GRAY))));
        }

        // The name of the location
        objs.add(new VisChain(LinAlg.rotateZ(Math.PI/2), LinAlg.translate(0,-.8,1.002),
                              LinAlg.scale(0.02),
                              new VzText(VzText.ANCHOR.CENTER, String.format("<<black>> %s", name))));

        return new VisChain(objs.toArray());
    }

    public Shape getShape()
    {
    	return new BoxShape(lwh[0]*2, lwh[1]*2, lwh[2]*2);
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public String getName()
    {
        return name;
    }

    public Obj getObj(boolean assignID)
    {
        Obj locObj;
        if(assignID && id < 0) {
            locObj = new Obj(assignID);
            id = locObj.getID();
        }
        else {
            locObj = new Obj(id);
        }
        
        lwh = new double[]{scale, scale, scale};

        double[] pose = LinAlg.matrixToXyzrpy(T);

        locObj.setPose(pose);
        locObj.setCentroid(new double[]{pose[0], pose[1], pose[2]});
        locObj.setBoundingBox(new BoundingBox(LinAlg.scale(lwh, 2), pose));

        locObj.setVisObject(getVisObject());
        locObj.setShape(getShape());
        locObj.setSourceSimObject(this);
        Classifications location = new Classifications();
        location.add(name, 1.0);
        locObj.addClassifications(FeatureCategory.LOCATION, location);

        return locObj;
    }

    public void read(StructureReader ins) throws IOException
    {
    	super.read(ins);

        name = ins.readString();
    }

    public void write(StructureWriter outs) throws IOException
    {
    	super.write(outs);

        outs.writeString(name);
    }

    // Override for SimObject
    public void setRunning(boolean arg0)
    {
    }
}
