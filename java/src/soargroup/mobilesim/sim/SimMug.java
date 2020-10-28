package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.rosie.RosieConstants;

import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.attributes.*;

public class SimMug extends RosieSimObject {
	protected Fillable fillable = null;

	public SimMug(SimWorld sw){
		super(sw);
	}

	@Override
	public void init(ArrayList<SimObject> worldObjects) { 
		String contents = properties.get(RosieConstants.CONTENTS);
		fillable = new Fillable(this, contents);
		addAttribute(fillable);

		super.init(worldObjects);
	}

	@Override
	public VisChain createVisObject(){
		String contents = fillable.getContents();
		VisChain vc = new VisChain();

		VzMesh.Style style = new VzMesh.Style(color);
		vc.add(new VzCylinder(0.5, 1.0, VzCylinder.BOTTOM, style));
		// Add a side box for the handle
		vc.add(new VisChain(LinAlg.translate(-0.6, 0.0, 0.0), new VzBox(0.2, 0.1, 0.6, style)));
		if(!contents.equals(RosieConstants.EMPTY)){
			// Add a circle for the liquid
			VzMesh.Style liquidStyle = new VzMesh.Style(new Color(100, 100, 20));
			vc.add(new VisChain(LinAlg.translate(0.0, 0.0, 0.4), new VzCircle(0.5, liquidStyle)));
		}
		return new VisChain(LinAlg.scale(scale_xyz[0], scale_xyz[1], scale_xyz[2]), vc);
	}
}
