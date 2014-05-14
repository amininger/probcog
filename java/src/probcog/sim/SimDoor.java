package probcog.sim; // XXX - Should it go here? XXX Probably not

import java.awt.Color;
import java.io.IOException;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

public class SimDoor implements SimObject
{
	protected double[][] T = LinAlg.identity(4);  // position
    protected Color color = new Color(51, 25, 0);
    protected double[] scale = new double[]{.8, 0.1, 2};

    public SimDoor(SimWorld sw)
    {
    }

    public void setPose(double[][] T)
    {
        this.T = T;
    }

    public double[][] getPose()
    {
        return T;
    }

    public VisObject getVisObject()
    {
        ArrayList<Object> objs = new ArrayList<Object>();

        objs.add(new VisChain(LinAlg.scale(scale[0], scale[1], scale[2]),
                              new VzBox(new VzMesh.Style(color))));

        return new VisChain(objs.toArray());
    }

    public Shape getShape()
    {
        BoxShape shape = new BoxShape(0, 0, 0);
        return shape.transform(LinAlg.translate(0, 0, 1));
        // return new BoxShape(scale[0], scale[1], scale[2]);
    }

    public void read(StructureReader ins) throws IOException
    {
    	// 6 doubles for pose information (XYZRPY)
        double xyzrpy[] = ins.readDoubles();
        this.T = LinAlg.xyzrpyToMatrix(xyzrpy);
    }

    public void write(StructureWriter outs) throws IOException
    {
        outs.writeDoubles(LinAlg.matrixToXyzrpy(T));
    }

    // Override for SimObject
    public void setRunning(boolean arg0)
    {
    }
}
