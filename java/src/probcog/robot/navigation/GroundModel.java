package magic.slam;

public class GroundModel
{
    // same conventions as GridMap
    public double x0, y0;
    public double metersPerPixel;
    public double data[]; // min value at every point
    public int width, height;

    double slope;

    public GroundModel(double x0, double y0, double sizex, double sizey, double metersPerPixel, double slope)
    {
        this.x0 = x0;
        this.y0 = y0;
        this.metersPerPixel = metersPerPixel;
        this.slope = slope;

        width = (int) (sizex / this.metersPerPixel + 1);
        height = (int) (sizey / this.metersPerPixel + 1);

        data = new double[width*height];
        for (int i = 0; i < data.length; i++) {
            data[i] = Double.MAX_VALUE;
        }
    }

    public void addPoint(double x, double y, double z)
    {
        int ix = (int) ((x - x0) / metersPerPixel);
        int iy = (int) ((y - y0) / metersPerPixel);
        if (ix >= 0 && ix < width && iy >= 0 && iy < height)
            data[iy*width+ix] = Math.min(data[iy*width+ix], z);
    }

    boolean didFinish = false;

    void finish()
    {
        if (didFinish)
            return;

        int maxiter = 10;

        for (int iter = 0; iter < maxiter; iter++) {
            boolean dirty = false;
            for (int iy = 1; iy+1 < height; iy++) {
                for (int ix = 1; ix+1 < width; ix++) {
                    double v = data[iy*width+ix];

                    for (int dy = -1; dy <= 1; dy++) {
                        for (int dx = -1; dx <= 1; dx++) {
                            double n = data[(iy+dy)*width + ix+dx];
                            if (n == Double.MAX_VALUE)
                                continue;

                            // dist inefficiently computed.
                            double dist;

                            dist = Math.sqrt(dy*dy + dx*dx);
                            dist *= metersPerPixel;

                            double nv = n + slope*dist;
                            if (nv < v) {
                                v = nv;
                                dirty = true;
                            }
                        }
                    }
                    data[iy*width+ix] = v;
                }
            }

            if (!dirty)
                break;
        }

        didFinish = true;
    }

    public double getMinGroundHeight()
    {
        finish();

        double v = Double.MAX_VALUE;
        for (int i = 0; i < data.length; i++)
            v = Math.min(v, data[i]);
        return v;
    }

    public double getGroundHeight(double x, double y)
    {
        finish();

        int ix = (int) ((x - x0) / metersPerPixel);
        int iy = (int) ((y - y0) / metersPerPixel);

        if (ix >= 0 && ix < width && iy >= 0 && iy < height)
            return data[iy*width+ix];

        return Double.MAX_VALUE;
    }
}
