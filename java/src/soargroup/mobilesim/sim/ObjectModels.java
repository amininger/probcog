package soargroup.mobilesim.sim;

import java.awt.Color;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;

public class ObjectModels {
	public static VisChain createModel(String modelName, double[] scale, Color color){
		modelName = modelName.toLowerCase();
		if(modelName.equals("chair")){
			return createChairModel(scale, color);
		}
		if(modelName.equals("bunk")){
			return createBunkModel(scale, color);
		}
		if(modelName.equals("shelf")){
			return createShelfModel(scale, color);
		}

		return null;
	}

	public static VisChain createChairModel(double[] scale, Color color){
		VisChain c = new VisChain();
		VzMesh.Style chair_color = new VzMesh.Style(color);
		VzMesh.Style leg_color = new VzMesh.Style(new Color(94, 76, 28)); // brown

		final double LEG_W = 0.1;
		final double SEAT_H = 0.1;
		final double BACK_W = 0.1;

		// Chair Back
		double back_height = scale[2]/2 - SEAT_H/2;
		c.add(new VisChain(LinAlg.translate(-scale[0]/2 + BACK_W/2, 0.0, SEAT_H/2 + back_height/2), 
					new VzBox(BACK_W, scale[1], back_height, chair_color)
		));

		// Seat
		c.add(new VzBox(scale[0], scale[1], SEAT_H, chair_color));

		double leg_h = (scale[2] - LEG_W)/2;

		double leg_x = (scale[0] - LEG_W)/2;
		double leg_y = (scale[1] - LEG_W)/2;
		double leg_z = -SEAT_H/2 - leg_h/2;

		// Legs of Chair
		c.add(new VisChain(LinAlg.translate(leg_x, leg_y, leg_z), new VzBox(LEG_W, LEG_W, leg_h, leg_color)));
		c.add(new VisChain(LinAlg.translate(-leg_x, leg_y, leg_z), new VzBox(LEG_W, LEG_W, leg_h, leg_color)));
		c.add(new VisChain(LinAlg.translate(-leg_x, -leg_y, leg_z), new VzBox(LEG_W, LEG_W, leg_h, leg_color)));
		c.add(new VisChain(LinAlg.translate(leg_x, -leg_y, leg_z), new VzBox(LEG_W, LEG_W, leg_h, leg_color)));
		return c;
	}

	public static VisChain createBunkModel(double[] scale, Color color){
		VisChain vc = new VisChain();

		final double POLE_W = 0.1;
		VzMesh.Style bunk_color = new VzMesh.Style(color); 

		// Corner Poles
		double pole_x = (scale[0] - POLE_W)/2;
		double pole_y = (scale[1] - POLE_W)/2;
		vc.add(new VisChain(LinAlg.translate( pole_x,  pole_y, 0.0), new VzBox(POLE_W, POLE_W, scale[2], bunk_color)));
		vc.add(new VisChain(LinAlg.translate( pole_x, -pole_y, 0.0), new VzBox(POLE_W, POLE_W, scale[2], bunk_color)));
		vc.add(new VisChain(LinAlg.translate(-pole_x,  pole_y, 0.0), new VzBox(POLE_W, POLE_W, scale[2], bunk_color)));
		vc.add(new VisChain(LinAlg.translate(-pole_x, -pole_y, 0.0), new VzBox(POLE_W, POLE_W, scale[2], bunk_color)));

		// Bottom Bunk
		vc.add(new VisChain(LinAlg.translate(0.0, 0.0, -scale[2]*0.25), new VzBox(scale[0], scale[1], 0.2, bunk_color)));

		// Top Bunk
		vc.add(new VisChain(LinAlg.translate(0.0, 0.0, scale[2]*0.25), new VzBox(scale[0], scale[1], 0.2, bunk_color)));

		return vc;
	}

	public static VisChain createShelfModel(double[] scale, Color color){
		VisChain c = new VisChain();
		VzMesh.Style style = new VzMesh.Style(color);

		final double SHELF_SPACING = 0.50; // vertical spacing between shelves

		// Outer Bounds
		c.add(new VisChain(LinAlg.rotateY(Math.PI/2), new VzOpenBox(scale[2], scale[1], scale[0], style)));

		// Shelves
		int nshelves = (int)Math.ceil(scale[2]/SHELF_SPACING)-1;
		double sz = -(nshelves-1)/2.0;
		for(int i = 0; i < nshelves; i += 1){
			c.add(new VisChain(LinAlg.translate(0.0, 0.0, (sz + i)*SHELF_SPACING), 
						new VzRectangle(scale[0], scale[1], new VzMesh.Style(color))));
		}

		return c;
	}
}
