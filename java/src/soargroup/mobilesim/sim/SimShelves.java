package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;
import soargroup.rosie.RosieConstants;

import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.attributes.*;

public class SimShelves extends SimBoxObject {
	private static final double SHELF_SPACING = 0.50; // vertical spacing between shelves
	private static final double OBJ_SPACING = 0.60;   // how far apart objects are placed on a shelf

	// Attributes
	protected Openable openable = null;
	protected Receptacle receptacle = null;

	public SimShelves(SimWorld sw){
		super(sw);
	}

	@Override
	public void init(ArrayList<SimObject> worldObjects) { 
		// Setup points to put objects on the shelves
		receptacle = new Receptacle(this, false);
		int nshelves = (int)Math.ceil(scale_xyz[2]/SHELF_SPACING)-1;
		double sz = -(nshelves-1)/2.0;
		for(int i = 0; i < nshelves; i += 1){
			receptacle.addPoints(scale_xyz[0], scale_xyz[1], (sz + i)*SHELF_SPACING, OBJ_SPACING);
		}

		// If there is a door property, then make it openable
		if(properties.containsKey(RosieConstants.DOOR)){
			boolean isOpen = properties.get(RosieConstants.DOOR).equals(RosieConstants.DOOR_OPEN);
			openable = new Openable(this, isOpen);
		}
	}

	@Override
	public VisChain createVisObject(){
		VisChain c = new VisChain();
		VzMesh.Style style = new VzMesh.Style(color);

		// Outer Bounds
		c.add(new VisChain(
			LinAlg.rotateY(Math.PI/2),
			new VzOpenBox(new double[]{ scale_xyz[2], scale_xyz[1], scale_xyz[0] }, style)
		));

		// Shelves
		int nshelves = (int)Math.ceil(scale_xyz[2]/SHELF_SPACING)-1;
		double sz = -(nshelves-1)/2.0;
		for(int i = 0; i < nshelves; i += 1){
			c.add(new VisChain(LinAlg.translate(0.0, 0.0, (sz + i)*SHELF_SPACING), 
						new VzRectangle(scale_xyz[0], scale_xyz[1], new VzMesh.Style(color))));
		}

		if(openable != null && !openable.isOpen()){
			// Door
			c.add(new VisChain(
				LinAlg.translate(scale_xyz[0]/2, 0.0, 0.0), LinAlg.rotateY(Math.PI/2), 
				new VzRectangle(scale_xyz[2], scale_xyz[1], new VzMesh.Style(color))));
		}

		return c;
	}

	// Action Handling Rules
	static {
		//// SetProp.Open Apply: Update the model to have an open drawer
		//ActionHandler.addApplyRule(SetProp.Open.class, new ActionHandler.ApplyRule<SetProp.Open>() {
		//	public Result apply(SetProp.Open open){
		//		if(open.object instanceof SimDrawer){
		//			((SimDrawer)open.object).setDoorPosition(open.value);
		//		}
		//		return Result.Ok();
		//	}
		//});
		//// SetProp.Close Apply: Make object collidable again
		//ActionHandler.addApplyRule(SetProp.Close.class, new ActionHandler.ApplyRule<SetProp.Close>() {
		//	public Result apply(SetProp.Close close){
		//		if(close.object instanceof SimDrawer){
		//			((SimDrawer)close.object).setDoorPosition(close.value);
		//		}
		//		return Result.Ok();
		//	}
		//});
	}
}
