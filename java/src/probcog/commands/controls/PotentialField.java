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
        widthPx = (int)(Math.ceil(width/metersPerPixel));
        heightPx = (int)(Math.ceil(height/metersPerPixel));

        widthPx += 4 - (widthPx%4);
        heightPx += 4 - (heightPx%4);

        data = new double[widthPx*heightPx];
    }

    /** Relative coordinates */
    public boolean inRange(double rx, double ry)
    {
        int ix = (int)(Math.floor(rx/metersPerPixel)) + widthPx/2;
        int iy = (int)(Math.floor(ry/metersPerPixel)) + heightPx/2;

        return inRange(ix, iy);
    }

    public boolean inRange(int ix, int iy)
    {
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
        int ix = (int)(Math.floor(rx/metersPerPixel)) + widthPx/2;
        int iy = (int)(Math.floor(ry/metersPerPixel)) + heightPx/2;
        return getIndex(ix, iy);
    }

    public double getIndex(int ix, int iy)
    {
        if (!inRange(ix, iy)) {
            System.err.println("ERR: Requested value outside of range");
            return Double.MAX_VALUE;
        }

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
        int ix = (int)(Math.floor(rx/metersPerPixel)) + widthPx/2;
        int iy = (int)(Math.floor(ry/metersPerPixel)) + heightPx/2;
        setIndex(ix, iy, v);
    }

    public void setIndex(int ix, int iy, double v)
    {
        if (!inRange(ix, iy)) {
            System.err.println("ERR: Cannot set value outside of range");
            return;
        }

        data[iy*widthPx + ix] = v;
    }

    public void addRelative(double rx, double ry, double v)
    {
        int ix = (int)(Math.floor(rx/metersPerPixel)) + widthPx/2;
        int iy = (int)(Math.floor(ry/metersPerPixel)) + heightPx/2;
        addIndex(ix, iy, v);
    }

    public void addIndex(int ix, int iy, double v)
    {
        double pv = getIndex(ix, iy);

        // Handle wraparound near max value. Since potentials should never be
        // negative, just trigger by looking for negative values?
        double nv = pv+v;
        if (Double.isInfinite(nv))
            nv = Double.MAX_VALUE;

        setIndex(ix, iy, nv);
    }

    /** Convert an index value to global coordinates in meters */
    public double[] indexToMeters(int ix, int iy)
    {
        double[] pxy = new double[] { (ix - widthPx/2 + 0.5)*metersPerPixel,
                                      (iy - heightPx/2 + 0.5)*metersPerPixel };

        return LinAlg.transform(origin, pxy);
    }

    public int getWidth()
    {
        return widthPx;
    }

    public int getHeight()
    {
        return heightPx;
    }

    public double getMPP()
    {
        return metersPerPixel;
    }

    public double getMaxValue()
    {
        double max = 0;
        for (double v: data)
            max = Math.max(max, v);
        return max;
    }

    public double getMinValue()
    {
        double min = Double.MAX_VALUE;
        for (double v: data)
            min = Math.min(min, v);
        return min;
    }

    /** Get a default grayscale image of the potential field */
    public VisChain getVisObject()
    {
        return getVisObject(null);
    }

    /** Assume that colormapper has already been swapped to ARBG if neccessary */
    public VisChain getVisObject(ColorMapper cm)
    {
        BufferedImage im;
        double maxValue = getMaxValue();
        double minValue = getMinValue();
        if (cm == null) {
            im = new BufferedImage(widthPx, heightPx, BufferedImage.TYPE_BYTE_GRAY);
            byte[] buf = ((DataBufferByte)(im.getRaster().getDataBuffer())).getData();

            for (int i = 0; i < data.length; i++) {
                if (maxValue > 0)
                    buf[i] = (byte)((data[i]/maxValue)*0xff);
            }
        } else {
            cm.setMinMax(minValue, maxValue);
            im = new BufferedImage(widthPx, heightPx, BufferedImage.TYPE_INT_ARGB);
            int[] buf = ((DataBufferInt)(im.getRaster().getDataBuffer())).getData();

            for (int i = 0; i < data.length; i++) {
                buf[i] = cm.map(data[i]);
            }
        }

        VzImage vim = new VzImage(im);
        return new VisChain(LinAlg.xytToMatrix(origin),
                            LinAlg.translate(-widthPx*0.5*metersPerPixel, -heightPx*0.5*metersPerPixel),
                            LinAlg.scale(metersPerPixel),
                            vim);
    }
}
