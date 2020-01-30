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

public class SimReceptacle extends SimBoxObject {
	public static final double ANCHOR_SPACING = 0.40;

	public SimReceptacle(SimWorld sw){
		super(sw);
	}

	@Override
	public VisChain createVisObject() {
		// Use an OpenBox instead of a regular box
		VisChain c = new VisChain();
		c.add(new VzOpenBox(scale_xyz, new VzMesh.Style(color)));
		return c;
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		// Create anchors inside on the bottom
		anchors = AnchorPoint.create(scale_xyz[0], scale_xyz[1], -scale_xyz[2]/2, ANCHOR_SPACING, this, RosieConstants.REL_IN);
	}
}
