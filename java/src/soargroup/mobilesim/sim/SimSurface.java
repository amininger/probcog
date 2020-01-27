package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

public class SimSurface extends SimBoxObject {
	public static final double ANCHOR_SPACING = 0.40;

	public SimSurface(SimWorld sw){
		super(sw);
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		// Create anchors on the top (+height/2)
		anchors = AnchorPoint.create(scale_xyz[0], scale_xyz[1], scale_xyz[2]/2, ANCHOR_SPACING, this, "on");
	}
}
