package probcog.commands;

import java.io.*;

import april.jmat.*;

/** Node value for command version of simple graph */
public class CommandNode implements Serializable
{
    double[] xy;

    public CommandNode()
    {
        this(new double[2]);
    }

    public CommandNode(double[] xy)
    {
        this.xy = LinAlg.resize(xy, 2);
    }

    public void setXY(double[] xy)
    {
        this.xy[0] = xy[0];
        this.xy[1] = xy[1];
    }

    public void setXY(double x, double y)
    {
        xy[0] = x;
        xy[1] = y;
    }

    public double[] getXY()
    {
        return xy;
    }
}
