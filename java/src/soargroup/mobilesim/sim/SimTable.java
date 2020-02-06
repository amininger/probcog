package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.sim.attributes.Surface;

public class SimTable extends RosieSimObject {
	private final double THICKNESS = 0.1;

	public SimTable(SimWorld sw){
		super(sw);
	}

	@Override
	public void init(ArrayList<SimObject> simObjects){
		super.init(simObjects);
		addAttribute(new Surface(this, true));
	}

	@Override
	public VisChain createVisObject(){
		VisChain c = new VisChain();
		VzMesh.Style style = new VzMesh.Style(color);

		double dX = (scale_xyz[0] - THICKNESS)/2;
		double dY = (scale_xyz[1] - THICKNESS)/2;
		double dZ = (scale_xyz[2] - THICKNESS)/2;

		// Top of table
		c.add(new VisChain(
			LinAlg.translate(0.0, 0.0, dZ), 
			new VzBox(new double[]{ scale_xyz[0], scale_xyz[1], THICKNESS }, style)
		));

		// Legs of table
		c.add(new VisChain(
			LinAlg.translate(dX, dY, -THICKNESS/2), 
			new VzBox(new double[]{ THICKNESS, THICKNESS, dZ*2 }, style)
		));
		c.add(new VisChain(
			LinAlg.translate(-dX, dY, -THICKNESS/2), 
			new VzBox(new double[]{ THICKNESS, THICKNESS, dZ*2 }, style)
		));
		c.add(new VisChain(
			LinAlg.translate(-dX, -dY, -THICKNESS/2), 
			new VzBox(new double[]{ THICKNESS, THICKNESS, dZ*2 }, style)
		));
		c.add(new VisChain(
			LinAlg.translate(dX, -dY, -THICKNESS/2), 
			new VzBox(new double[]{ THICKNESS, THICKNESS, dZ*2 }, style)
		));
		return c;
	}
}
