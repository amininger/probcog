package probcog.old.sim;

import java.awt.Color;
import java.io.IOException;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.util.*;

public class SimDoor implements SimObject
{
	protected double[][] T = LinAlg.identity(4);  // position
    protected Color color = new Color(51, 25, 0);
    protected double[] scale = new double[]{.9, 0.1, 1.5};

    protected int id;

    // Characteristics of classification confidence distribution
    protected double mean = 0.9;
    protected double stddev = 0.05;

    public SimDoor(SimWorld sw)
    {
        id = Util.nextID();
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
                              LinAlg.translate(0, 0, scale[2]/9),
                              new VzBox(new VzMesh.Style(color))));

        return new VisChain(objs.toArray());
    }

    public Shape getShape()
    {
        // BoxShape shape = new BoxShape(0, 0, 0);
        // return shape.transform(LinAlg.translate(0, 0, 1));
        return new BoxShape(scale[0], scale[1], -scale[2]); // Negative z scale makes box invisible to LIDAR
    }

    public void read(StructureReader ins) throws IOException
    {
    	// 6 doubles for pose information (XYZRPY)
        double xyzrpy[] = ins.readDoubles();
        this.T = LinAlg.xyzrpyToMatrix(xyzrpy);

        // IDs are automatically generated upon creation. Right now, don't
        // care which door has which ID.

        this.mean = ins.readDouble();
        this.stddev = ins.readDouble();
    }

    public void write(StructureWriter outs) throws IOException
    {
        outs.writeDoubles(LinAlg.matrixToXyzrpy(T));
        outs.writeComment("mean and stddev of confidence measure");
        outs.writeDouble(mean);
        outs.writeDouble(stddev);
    }

    // Override for SimObject
    public void setRunning(boolean arg0)
    {
    }
}
