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

    public SimLocation(SimWorld sw)
    {
    	super(sw);
    }

    public VisObject getVisObject()
    {
        ArrayList<Object> objs = new ArrayList<Object>();
        
        objs.add(LinAlg.scale(scale));

		Color fillColor = color;
        if(currentState.containsKey(SoarConcepts.ACTIVATION) && currentState.get(SoarConcepts.ACTIVATION).equals(SoarConcepts.ON)) {
			fillColor = new Color(
					(int)((fillColor.getRed() + 255)/2),
					(int)((fillColor.getGreen() + 255)/2),
					(int)((fillColor.getBlue() + 255)/2));
		}
        // The larger box making up the background of the object
        objs.add(new VisChain(LinAlg.translate(0, 0, .5), new VzRectangle(lenxyz[0], lenxyz[1], new VzMesh.Style(fillColor))));

        // The smaller inner box is only drawn if there is a door and it's open
        if(currentState.containsKey(SoarConcepts.DOOR) && currentState.get(SoarConcepts.DOOR).equals(SoarConcepts.OPEN)) {
            objs.add(new VisChain(LinAlg.translate(0,0,0.501),
                                  LinAlg.scale(.9),
                                  new VzRectangle(lenxyz[0], lenxyz[1], new VzMesh.Style(Color.DARK_GRAY))));
        }

        // The name of the location
        objs.add(new VisChain(LinAlg.rotateZ(Math.PI/2), LinAlg.translate(0,-.4,0.502),
                              LinAlg.scale(0.015), 
                              new VzText(VzText.ANCHOR.CENTER, String.format("<<black>> %s", name))));

        return new VisChain(objs.toArray());
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public String getName()
    {
        return name;
    }

    @Override
    public Obj getObj()
    {
    	Obj obj = super.getObj();
    	
        Classifications location = new Classifications();
        location.add(name, 1.0);
        obj.addClassifications(FeatureCategory.LOCATION, location);
        
    	return obj;
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
