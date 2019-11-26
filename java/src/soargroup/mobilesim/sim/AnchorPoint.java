package soargroup.mobilesim.sim;

import java.util.ArrayList;
import april.jmat.LinAlg;

// An anchor is a point on an object where another object can be placed
// Used so that something like a table can have multiple objects not all at the same place
public class AnchorPoint {
	// Creates a set of points centered at 0, 0 that fit onto a rectange of the given dx and dy size
	//   at the given height and spacing
	public static ArrayList<AnchorPoint> create(double dx, double dy, double height, double spacing){
		ArrayList<AnchorPoint> anchors = new ArrayList<AnchorPoint>();
		int nrows = (int)Math.ceil(dy/spacing)-1;
		int ncols = (int)Math.ceil(dx/spacing)-1;
		double srow = -(nrows-1)/2.0;
		double scol = -(ncols-1)/2.0;
		for(int r = 0; r < nrows; r += 1){
			for(int c = 0; c < ncols; c += 1){
				anchors.add(new AnchorPoint((scol+c)*spacing, (srow+r)*spacing, height));
			}
		}
		return anchors;
	}
	public AnchorPoint(double x, double y, double z){
		this.xyz = new double[]{ x, y, z };
	}
	public double[] xyz;
	public RosieSimObject object = null;
}
