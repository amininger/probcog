package probcog.classify;

import java.util.*;

import april.jmat.*;

import probcog.util.*;

/** An entry-coordinate pair for classification,
 */
public class CPoint
{
    public String label;
    public double[] coords;

    public CPoint(String label_, ArrayList<Double> coords_)
    {
        this(label_, Util.toArray(coords_, new double[coords_.size()]));
    }

    public CPoint(String label_, double[] coords_)
    {
        label = label_;
        coords = coords_;
    }
    
    public void print(){
    	for(int i = 0; i < coords.length; i++){
    		System.out.print(String.format("%.3f ", coords[i]));
    	}
    	System.out.println("");
    }
}
