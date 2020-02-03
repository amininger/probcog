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

public class SimShelves extends SimBoxObject {
	private static final double SHELF_SPACING = 0.50;

	private String door;
	private boolean useDoor = false;
	private boolean isOpen = true;

	public SimShelves(SimWorld sw){
		super(sw);
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

		if(useDoor && !isOpen){
			// Door
			c.add(new VisChain(
				LinAlg.translate(scale_xyz[0]/2, 0.0, 0.0), LinAlg.rotateY(Math.PI/2), 
				new VzRectangle(scale_xyz[2], scale_xyz[1], new VzMesh.Style(color))));
		}

		return c;
	}

	@Override
	public void setState(String property, String value){
		super.setState(property, value);
		if(property.equals(RosieConstants.DOOR)){
			isOpen = value.equals(RosieConstants.DOOR_OPEN);
			recreateVisObject();
		}
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		if(properties.containsKey(RosieConstants.DOOR)){
			useDoor = true;
			isOpen = properties.get(RosieConstants.DOOR).equals(RosieConstants.DOOR_OPEN);
			recreateVisObject();
		}

		//anchors = new ArrayList<AnchorPoint>();
		//int nshelves = (int)Math.ceil(scale_xyz[2]/SHELF_SPACING)-1;
		//double sz = -(nshelves-1)/2.0;
		//for(int i = 0; i < nshelves; i += 1){
		//	anchors.addAll(AnchorPoint.create(scale_xyz[0], scale_xyz[1], (sz + i)*SHELF_SPACING, ANCHOR_SPACING, this, RosieConstants.REL_IN));
		//}
	}
}
