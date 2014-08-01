package probcog.sim;

import java.awt.Color;
import java.io.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.util.*;

/** Merely an object which produces an ID. Later, this can be mapped to a
 *  tag config file for identification as an arbitrary object.
 */
public class SimAprilTag implements SimObject
{
    double[][] T = LinAlg.identity(4);

    SimWorld sw;
    int id = 0;
    VisObject obj;
    Shape shape;

    public SimAprilTag(SimWorld sw)
    {
        this.sw = sw;
        id = Util.nextID();
        updateVisObject();

        shape = new BoxShape(0.5, 0.5, -0.1);
    }

    public int getID()
    {
        return id;
    }

    public void setPose(double[][] T)
    {
        this.T = LinAlg.copy(T);
    }

    public double[][] getPose()
    {
        return T;
    }

    public VisObject getVisObject()
    {
        return obj;
    }

    private void updateVisObject()
    {
        obj = new VisChain(new VzRectangle(0.5, 0.5, new VzMesh.Style(Color.magenta)),
                           LinAlg.scale(0.005),
                           new VzText(VzText.ANCHOR.CENTER, "<<dropshadow=false,monospaced-128>>"+id));
    }

    public Shape getShape()
    {
        return shape;
    }

    public void read(StructureReader ins) throws IOException
    {
    	// 6 doubles for pose information (XYZRPY)
        double xyzrpy[] = ins.readDoubles();
        this.T = LinAlg.xyzrpyToMatrix(xyzrpy);

        // ID Consistency IS important, since we want to map to config later.
        // Randomly generated the first time, but loaded from file later.
        this.id = ins.readInt();
        updateVisObject();
    }

    public void write(StructureWriter outs) throws IOException
    {
        outs.writeDoubles(LinAlg.matrixToXyzrpy(T));
        outs.writeInt(id);
    }

    // Override for SimObject
    public void setRunning(boolean arg0)
    {
    }
}
