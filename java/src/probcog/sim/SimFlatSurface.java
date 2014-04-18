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

public class SimFlatSurface extends SimObjectPC
{
    protected double[] lwh = new double[]{1, 1, 1};

    public SimFlatSurface(SimWorld sw)
    {
    	super(sw);
    }

    public VisObject getVisObject()
    {
        ArrayList<Object> objs = new ArrayList<Object>();
        
        objs.add(LinAlg.scale(lwh[0], lwh[1], lwh[2]));

        // The larger box making up the background of the object
        objs.add(new VisChain(LinAlg.translate(0, 0, -.01), new VzRectangle(new VzMesh.Style(Color.white))));

        return new VisChain(objs.toArray());
    }

    public Shape getShape()
    {
    	return new BoxShape(lwh[0]*2, lwh[1]*2, lwh[2]*2);
    }

    public void read(StructureReader ins) throws IOException
    {
    	lwh[0] = ins.readDouble();
    	lwh[1] = ins.readDouble();
    	lwh[2] = 0.001;
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeDouble(lwh[0]);
    	outs.writeDouble(lwh[1]);
    }

    // Override for SimObject
    public void setRunning(boolean arg0)
    {
    }
}
