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
    public SimFlatSurface(SimWorld sw)
    {
    	super(sw);
    }
    
    @Override
    public Shape getShape(){
    	return new SphereShape(0.0);
    }

    public VisObject getVisObject()
    {
    	return new VisChain(
    			LinAlg.scale(lenxyz[0], lenxyz[1], lenxyz[2]),
    			new VzRectangle(new VzMesh.Style(Color.white))
    	);
    }

    public void read(StructureReader ins) throws IOException
    {
    	lenxyz[0] = ins.readDouble();
    	lenxyz[1] = ins.readDouble();
    	lenxyz[2] = 0.001;
    	xyzrpy[2] = -0.001;
    }

    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeDouble(lenxyz[0]);
    	outs.writeDouble(lenxyz[1]);
    }

    // Override for SimObject
    public void setRunning(boolean arg0)
    {
    }
}
