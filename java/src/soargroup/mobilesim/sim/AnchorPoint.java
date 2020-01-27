package soargroup.mobilesim.sim;

import java.util.ArrayList;
import april.jmat.LinAlg;

// An anchor is a point on an object where another object can be placed
// Used so that something like a table can have multiple objects not all at the same place
public class AnchorPoint {
	// Creates a set of points centered at 0, 0 that fit onto a rectange of the given dx and dy size
	//   at the given height and spacing
	public static ArrayList<AnchorPoint> create(double dx, double dy, double height, double spacing, RosieSimObject parent, String relation){
		ArrayList<AnchorPoint> anchors = new ArrayList<AnchorPoint>();
		int nrows = (int)Math.ceil(dy/spacing)-1;
		int ncols = (int)Math.ceil(dx/spacing)-1;
		double srow = -(nrows-1)/2.0;
		double scol = -(ncols-1)/2.0;
		for(int r = 0; r < nrows; r += 1){
			for(int c = 0; c < ncols; c += 1){
				anchors.add(new AnchorPoint((scol+c)*spacing, (srow+r)*spacing, height, parent, relation));
			}
		}
		return anchors;
	}
	public AnchorPoint(double x, double y, double z, RosieSimObject parent, String relation){
		this.xyz = new double[]{ x, y, z };
		this.parent = parent;
		this.relation = relation;
	}
	public double[] xyz;
	public RosieSimObject parent = null;
	public RosieSimObject object = null;
	public String relation;

	// Gets the target pose of an object if it were to be added to this anchor
	public double[][] calcObjectPose(RosieSimObject obj){
		double[][] local_pose = LinAlg.translate(xyz[0], xyz[1], xyz[2] + obj.getScale()[2]/2 + 0.001);
		return LinAlg.matrixAB(parent.getPose(), local_pose);
	}

	public void addObject(RosieSimObject obj){
		this.object = obj;
		obj.setPose(calcObjectPose(obj));
	}

	public boolean hasObject(){
		return (object != null);
	}

	public void checkObject(){
		if(object == null){ return; }

		double[][] obj_pose = calcObjectPose(object);
		double[] obj_pos = LinAlg.matrixToXyzrpy(obj_pose);
		if(LinAlg.squaredDistance(obj_pos, object.getXYZRPY(), 2) > 0.01){
			// Object must have been moved
			object = null;
		}
	}

	public void move(){
		if(object != null){
			object.setPose(calcObjectPose(object));
		}
	}
}
