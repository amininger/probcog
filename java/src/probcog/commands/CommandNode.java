package probcog.commands;

import java.io.*;

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
        this.xy = xy;
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
