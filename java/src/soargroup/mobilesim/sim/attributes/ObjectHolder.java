package soargroup.mobilesim.sim.attributes;

import java.awt.Color;
import java.util.List;
import java.util.ArrayList;
import april.jmat.LinAlg;
import april.vis.*;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;
import soargroup.mobilesim.util.ResultTypes.*;

public class ObjectHolder extends Attribute {
	// Anchors are locations where objects can be placed
	protected ArrayList<AnchorPoint> anchors;
	public ObjectHolder(RosieSimObject object){
		super(object);
		this.anchors = anchors;
	}

	// Creates a set of points centered at 0, 0 that fit onto a rectange of the given dx and dy size
	//   at the given z height and spacing
	public void addPoints(double dx, double dy, double z, double spacing){
		int nrows = (int)Math.ceil(dy/spacing)-1;
		int ncols = (int)Math.ceil(dx/spacing)-1;
		double srow = -(nrows-1)/2.0;
		double scol = -(ncols-1)/2.0;
		for(int r = 0; r < nrows; r += 1){
			for(int c = 0; c < ncols; c += 1){
				anchors.add(new AnchorPoint((scol+c)*spacing, (srow+r)*spacing, z));
			}
		}
	}

	public void addPoint(double[] xyz){
		anchors.add(new AnchorPoint(xyz[0], xyz[1], xyz[2]));
	}

	@Override
	public void moveHandler(double[] xyzrpy){
		for(AnchorPoint pt : this.anchors){
			pt.updateHeldObject();
		}
	}

	@Override
	public void render(VisChain vc){
		// Add a small wireframe box for each anchor
		for(AnchorPoint anchor : anchors){
			vc.add(new VisChain(LinAlg.translate(anchor.xyz),
						new VzBox(new double[]{ 0.04, 0.04, 0.04}, new VzLines.Style(Color.black, 0.01))));
		}
	}

	// Registering Action Handling Rules
	@Override
	protected void setupRules(){
		//  PlaceObject: Valid if there is a free anchor available
		ActionHandler.addValidateRule(PlaceObject.class, new ValidateRule<PlaceObject>() {
			public IsValid validate(PlaceObject place){
				for(AnchorPoint pt : anchors){
					pt.checkObject();
					if(!pt.hasObject()){
						return IsValid.True();
					}
				}
				return IsValid.False("ObjectHolder: No free anchors");
			}
		});

		// PlaceObject Apply: Add the object to the anchors
		ActionHandler.addApplyRule(PlaceObject.class, new ApplyRule<PlaceObject>() {
			public Result apply(PlaceObject place){
				for(AnchorPoint pt : anchors){
					pt.checkObject();
					if(!pt.hasObject()){
						pt.addObject(place.object);
						return Result.Ok();
					}
				}
				return Result.Err("ObjectHolder: No free anchors");
			}
		});

		// PickUp Apply: Remove the object from any anchors
		ActionHandler.addApplyRule(PickUp.class, new ApplyRule<PickUp>() {
			public Result apply(PickUp pickup){
				for(AnchorPoint pt : anchors){
					if(pt.heldObj == pickup.object){
						pt.heldObj = null;
					}
				}
				return Result.Ok();
			}
		});
	}

	// An anchor is a point on an object where another object can be placed
	// Used so that something like a table can have multiple objects not all at the same place
	protected class AnchorPoint {
		public AnchorPoint(double x, double y, double z){
			this.xyz = new double[]{ x, y, z };
		}
		public double[] xyz;
		public RosieSimObject heldObj = null;

		public boolean hasObject(){
			return (heldObj != null);
		}

		public void addObject(RosieSimObject obj){
			heldObj = obj;
			heldObj.setPose(calcObjectPose());
		}

		// Makes sure the heldObject is still at the anchor point (hasn't been moved somewhere else)
		public void checkObject(){
			if(heldObj == null){ return; }

			double[][] obj_pose = calcObjectPose();
			double[] obj_pos = LinAlg.matrixToXyzrpy(obj_pose);
			if(LinAlg.squaredDistance(obj_pos, heldObj.getXYZRPY(), 2) > 0.01){
				// Object must have been moved
				heldObj = null;
			}
		}

		// Checks the held object, then updates its pose
		public void updateHeldObject(){
			checkObject();
			if(heldObj != null){
				heldObj.setPose(calcObjectPose());
			}
		}

		// Gets the pose of the held object at the anchor point (in world coordinates, apply parent transform)
		private double[][] calcObjectPose(){
			double[][] local_pose = LinAlg.translate(xyz[0], xyz[1], xyz[2] + heldObj.getScale()[2]/2 + 0.001);
			return LinAlg.matrixAB(object.getPose(), local_pose);
		}
	}
}

