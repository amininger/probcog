package probcog.commands.controls;

import java.awt.image.*;

import april.jmat.*;
import april.vis.*;

public class PotentialField
{
    int widthPx, heightPx;
    double metersPerPixel;
    double[] data;
    double[] origin;
    double[] invOrigin;

    /** Assumed to be centered around current robot pose. Operates similarly
     *  to a grid map, but can hold double values.
     **/
    public PotentialField(double[] robotXYT,
                          double width,
                          double height,
                          double metersPerPixel)
    {
        origin = LinAlg.copy(robotXYT);
        invOrigin = LinAlg.xytInverse(origin);

        this.metersPerPixel = metersPerPixel;
        int widthPx = (int)(Math.ceil(width/metersPerPixel));
        int heightPx = (int)(Math.ceil(height/metersPerPixel));
        data = new double[widthPx*heightPx];
    }

    /** Relative coordinates */
    public boolean inRange(double rx, double ry)
    {
        int ix = (int)(Math.floor(rx/metersPerPixel)) + widthPx/2;
        int iy = (int)(Math.floor(ry/metersPerPixel)) + heightPx/2;

        return ix >= 0 && ix < widthPx && iy >= 0 && ix < heightPx;
    }

    /** Retrieve a value in absolute coordinates */
    public double get(double x, double y)
    {
        double[] xy = new double[] {x, y};
        double[] rxy = LinAlg.transform(invOrigin, xy);
        return getRelative(rxy[0], rxy[1]);
    }

    /** Retrieve a value in coordinates relative to the robot */
    public double getRelative(double rx, double ry)
    {
        if (!inRange(rx, ry)) {
            System.err.println("ERR: Requested value outside of range");
            return Double.MAX_VALUE;
        }

        int ix = (int)(Math.floor(rx/metersPerPixel)) + widthPx/2;
        int iy = (int)(Math.floor(ry/metersPerPixel)) + heightPx/2;

        return data[iy*widthPx + ix];
    }

    /** Set a value in absolute coordinates */
    public void set(double x, double y, double v)
    {
        double[] xy = new double[] {x, y};
        double[] rxy = LinAlg.transform(invOrigin, xy);
        setRelative(rxy[0], rxy[1], v);
    }

    /** Set a value in coordinates relative to the robot */
    public void setRelative(double rx, double ry, double v)
    {
        if (!inRange(rx, ry)) {
            System.err.println("ERR: Cannot set value outside of range");
            return;
        }

        int ix = (int)(Math.floor(rx/metersPerPixel)) + widthPx/2;
        int iy = (int)(Math.floor(ry/metersPerPixel)) + heightPx/2;

        data[iy*widthPx + ix] = v;
    }

    public VisChain getVisObject()
    {
        return getVisObject(null);
    }

    /** Assume that colormapper has already been swapped to ARBG if neccessary */
    public VisChain getVisObject(ColorMapper cm)
    {
        BufferedImage im;
        if (cm == null) {
            im = new BufferedImage(widthPx, heightPx, BufferedImage.TYPE_BYTE_GRAY);
            byte[] buf = ((DataBufferByte)(im.getRaster().getDataBuffer())).getData();

            for (int i = 0; i < data.length; i++) {
                buf[i] = (byte)(data[i]*0xff);
            }
        } else {
            im = new BufferedImage(widthPx, heightPx, BufferedImage.TYPE_INT_ARGB);
            int[] buf = ((DataBufferInt)(im.getRaster().getDataBuffer())).getData();

            for (int i = 0; i < data.length; i++) {
                buf[i] = cm.map(data[i]);
            }
        }

        VzImage vim = new VzImage(im, VzImage.FLIP);
        return new VisChain(LinAlg.xytToMatrix(origin), vim);
    }
}
