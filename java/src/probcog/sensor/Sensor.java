package probcog.sensor;

import java.util.ArrayList;

public interface Sensor
{
    /** Get the number of pixels high the data will be.**/
    public int getHeight();

    /** Get the number of pixels wide the data will be.**/
    public int getWidth();
    
    /** Gets all the colored, 3D points in the view region. **/
    public ArrayList<double[]> getAllXYZRGB(boolean fastScan);

    /** Get a colored, 3D point of an object.**/
    public double[] getXYZRGB(int ix, int iy);

    /** Get the matrix that transforms from camera coords to world coords */
    public double[][] getCameraXform();
    
    public boolean stashFrame();
}
