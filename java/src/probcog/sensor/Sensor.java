package probcog.sensor;

public interface Sensor
{
    /** Get the number of pixels high the data will be.**/
    public int getHeight();

    /** Get the number of pixels wide the data will be.**/
    public int getWidth();

    /** Get a colored, 3D point of an object.**/
    public double[] getXYZRGB(int ix, int iy);

    public boolean stashFrame();
}
