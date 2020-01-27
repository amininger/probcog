package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;

public class SimDrawer extends SimReceptacle {
	public static final double ANCHOR_SPACING = 0.40;

	private String door;
	private boolean isOpen = false;

	public SimDrawer(SimWorld sw){
		super(sw);
		anchors = new ArrayList<AnchorPoint>();
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

	@Override
	public void setState(String property, String value){
		super.setState(property, value);
		if(property.equals("door2")){
			boolean newOpen = value.equals("open2");
			if(newOpen != isOpen){
				isOpen = newOpen;
				double[][] pose = this.getPose();
				double[][] translate = LinAlg.translate(isOpen ? scale_xyz[0] : -scale_xyz[0], 0.0, 0.0);
				pose = LinAlg.matrixAB(pose, translate);
				this.setPose(pose);

				recreateVisObject();
			}
		}
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		// [String] Door State: << open closed >>
		String door = ins.readString().toLowerCase();

		anchors = AnchorPoint.create(ANCHOR_SPACING*1.1, scale_xyz[1], -scale_xyz[2]/3, ANCHOR_SPACING, this, "in");
		properties.put("door2", "closed2");
		if(door.equals("open")){

		}
	}

	@Override
	public void write(StructureWriter outs) throws IOException
	{
		super.write(outs);
		outs.writeString(door);
	}
}
