package probcog.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.util.*;

// XXX This is not intended to be permanent, but acts as a proxy for an actual
// hallway detector one might run on a robot.
/** Marks the transition from one hallway into another. */
public class SimHallway implements SimObject
{
	protected double[][] T = LinAlg.identity(4);  // position
    protected Color color = new Color(100, 150, 100);
    double[] sxyz = new double[]{0.1, 2.0, 1.0};
    double orientation = 0; // [-PI, PI]

    protected int id;

    // Characteristics of classification confidence distribution
    protected double mean = 0.95;
    protected double stddev = 0.05;

    public SimHallway(SimWorld sw)
    {
        id = Util.nextID();
    }
    
    public int getID(){
    	return id;
    }
    
    public double getMean(){
    	return mean;
    }
    
    public double getStdDev(){
    	return stddev;
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
        VisVertexData vvd = new VisVertexData(new double[2], new double[] {1.0, 0});
        return new VisChain(new VzLines(vvd, VzLines.LINES, new VzLines.Style(Color.blue, 2)),
                            LinAlg.scale(sxyz[0], sxyz[1], 1),
                            new VzRectangle(new VzMesh.Style(color)));
    }

    public Shape getShape()
    {
        // BoxShape shape = new BoxShape(0, 0, 0);
        // return shape.transform(LinAlg.translate(0, 0, 1));
        return new BoxShape(sxyz[0], sxyz[1], -sxyz[2]); // Negative z sxyz makes box invisible to LIDAR
    }

    public void read(StructureReader ins) throws IOException
    {
    	// 6 doubles for pose information (XYZRPY)
        double xyzrpy[] = ins.readDoubles();
        this.T = LinAlg.xyzrpyToMatrix(xyzrpy);
        this.sxyz = ins.readDoubles();

        // IDs are automatically generated upon creation. Right now, don't
        // care which door has which ID.

        this.mean = ins.readDouble();
        this.stddev = ins.readDouble();
    }

    public void write(StructureWriter outs) throws IOException
    {
        outs.writeComment("Pose XYZRPY");
        outs.writeDoubles(LinAlg.matrixToXyzrpy(T));
        outs.writeComment("Object scale");
        outs.writeDoubles(sxyz);
        outs.writeComment("mean and stddev of confidence measure");
        outs.writeDouble(mean);
        outs.writeDouble(stddev);
    }

    // Override for SimObject
    public void setRunning(boolean arg0)
    {
    }
}
