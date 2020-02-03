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

public class SimDrawer extends SimBoxObject {
	public static final double ANCHOR_SPACING = 0.40;

	private String door;
	private boolean isOpen = false;

	public SimDrawer(SimWorld sw){
		super(sw);
	}

	@Override
	public VisChain createVisObject() {
		VisChain c = new VisChain();

		if(isOpen){
			// Draw open drawer
			c.add(new VzOpenBox(scale_xyz, new VzMesh.Style(color)));
			// Draw body of drawer
			c.add(new VisChain(LinAlg.translate(-scale_xyz[0], 0.0, 0.0), 
						new VzBox(scale_xyz, new VzMesh.Style(color))));
		} else {
			c.add(new VzBox(scale_xyz, new VzMesh.Style(color)));
		}

		return c;
	}

	private void setDoorPosition(boolean open){
		if(open != isOpen){
			double[][] pose = this.getPose();
			double[][] translate = LinAlg.translate(isOpen ? scale_xyz[0] : -scale_xyz[0], 0.0, 0.0);
			pose = LinAlg.matrixAB(pose, translate);
			this.setPose(pose);

			recreateVisObject();
			isOpen = open;
		}
	}

	@Override
	public void setState(String property, String value){
		super.setState(property, value);
		if(property.equals(RosieConstants.DOOR)){
			boolean newOpen = value.equals(RosieConstants.DOOR_OPEN);
			setDoorPosition(newOpen);
		}
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);

		//anchors = AnchorPoint.create(ANCHOR_SPACING*1.1, scale_xyz[1], -scale_xyz[2]/3, ANCHOR_SPACING, this, "in");

		boolean newOpen = properties.get(RosieConstants.DOOR).equals(RosieConstants.DOOR_OPEN);
		setDoorPosition(newOpen);
	}
}
