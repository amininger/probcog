package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;

public class SimShelves extends SimReceptacle {
	private static final double SHELF_SPACING = 0.50;
	private VisObject visObject = null;

	private String door;
	private boolean useDoor = false;
	private boolean isOpen = true;
	private boolean staleVis = true;

	public SimShelves(SimWorld sw){
		super(sw);
	}

	private VisObject createVisObject(){
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

		if(!isOpen){
			// Door
			c.add(new VisChain(
				LinAlg.translate(scale_xyz[0]/2, 0.0, 0.0), LinAlg.rotateY(Math.PI/2), 
				new VzRectangle(scale_xyz[2], scale_xyz[1], new VzMesh.Style(color))));
		}

		// Uncomment to also draw anchor points
		//for(AnchorPoint anchor : anchors){
		//	c.add(new VisChain(LinAlg.translate(anchor.xyz), 
		//				new VzBox(new double[]{ 0.04, 0.04, 0.04}, new VzMesh.Style(Color.black))));
		//}
		return c;
	}

	@Override
	public VisObject getVisObject(){
		if(staleVis){
			visObject = createVisObject();
		}
		return visObject;
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		// [String] Door State: << open closed none >>
		door = ins.readString().toLowerCase();
		if(door.equals("none")){
			useDoor = false;
			isOpen = true;
		} else {
			useDoor = true;
			isOpen = (door.equals("open"));
			properties.put("door2", isOpen ? "open2" : "closed2");
		}

		anchors = new ArrayList<AnchorPoint>();
		int nshelves = (int)Math.ceil(scale_xyz[2]/SHELF_SPACING)-1;
		double sz = -(nshelves-1)/2.0;
		for(int i = 0; i < nshelves; i += 1){
			anchors.addAll(AnchorPoint.create(scale_xyz[0], scale_xyz[1], (sz + i)*SHELF_SPACING, ANCHOR_SPACING));
		}
	}

	@Override
	public void write(StructureWriter outs) throws IOException
	{
		super.write(outs);
		outs.writeString(door);
	}
}
