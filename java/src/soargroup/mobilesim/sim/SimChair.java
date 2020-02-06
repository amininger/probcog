package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

public class SimChair extends RosieSimObject {
	public SimChair(SimWorld sw){
		super(sw);
	}

	@Override
	public VisChain createVisObject(){
		VisChain c = new VisChain();
		VzMesh.Style chair_color = new VzMesh.Style(color);
		VzMesh.Style leg_color = new VzMesh.Style(new Color(94, 76, 28)); // brown

		final double LEG_W = 0.1;
		final double SEAT_H = 0.1;
		final double BACK_W = 0.1;

		// Chair Back
		double back_height = scale_xyz[2]/2 - SEAT_H/2;
		c.add(new VisChain(LinAlg.translate(-scale_xyz[0]/2 + BACK_W/2, 0.0, SEAT_H/2 + back_height/2), 
					new VzBox(new double[]{ BACK_W, scale_xyz[1], back_height }, chair_color)
		));

		// Seat
		c.add(new VzBox(new double[]{ scale_xyz[0], scale_xyz[1], SEAT_H }, chair_color));

		double leg_h = (scale_xyz[2] - LEG_W)/2;

		double leg_x = (scale_xyz[0] - LEG_W)/2;
		double leg_y = (scale_xyz[1] - LEG_W)/2;
		double leg_z = -SEAT_H/2 - leg_h/2;

		// Legs of Chair
		c.add(new VisChain(
			LinAlg.translate(leg_x, leg_y, leg_z), 
			new VzBox(new double[]{ LEG_W, LEG_W, leg_h }, leg_color)
		));
		c.add(new VisChain(
			LinAlg.translate(-leg_x, leg_y, leg_z), 
			new VzBox(new double[]{ LEG_W, LEG_W, leg_h }, leg_color)
		));
		c.add(new VisChain(
			LinAlg.translate(-leg_x, -leg_y, leg_z), 
			new VzBox(new double[]{ LEG_W, LEG_W, leg_h }, leg_color)
		));
		c.add(new VisChain(
			LinAlg.translate(leg_x, -leg_y, leg_z), 
			new VzBox(new double[]{ LEG_W, LEG_W, leg_h }, leg_color)
		));
		return c;
	}
}
