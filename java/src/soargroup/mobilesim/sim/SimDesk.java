package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

public class SimDesk extends SimBoxObject {
	private final double THICKNESS = 0.1;
	private VisObject visObject = null;

	public SimDesk(SimWorld sw){
		super(sw);
	}

	@Override
	public VisChain createVisObject(){
		VisChain c = new VisChain();
		VzMesh.Style style = new VzMesh.Style(color);

		double dX = (scale_xyz[0] - THICKNESS)/2;
		double dY = (scale_xyz[1] - THICKNESS)/2;
		double dZ = (scale_xyz[2] - THICKNESS)/2;

		// Top of desk
		c.add(new VisChain(
			LinAlg.translate(0.0, 0.0, dZ), 
			new VzBox(new double[]{ scale_xyz[0], scale_xyz[1], THICKNESS }, style)
		));

		// Side of desk
		c.add(new VisChain(
			LinAlg.translate(0.0, dY, -THICKNESS/2), 
			new VzBox(new double[]{ scale_xyz[0], THICKNESS, dZ*2 }, style)
		));

		// Drawer of desk
		c.add(new VisChain(
			LinAlg.translate(0.0, -scale_xyz[1]/4.0, -THICKNESS/2), LinAlg.rotateY(Math.PI/2.0), 
			new VzBox(new double[]{ dZ*2, scale_xyz[1]/2.0, scale_xyz[0] }, style)
		));
		return c;
	}
}
