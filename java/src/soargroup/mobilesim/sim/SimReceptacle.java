package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;

public class SimReceptacle extends SimBoxObject {
	protected ArrayList<AnchorPoint> anchors;
	public static final double ANCHOR_SPACING = 0.40;

	public SimReceptacle(SimWorld sw){
		super(sw);
		anchors = new ArrayList<AnchorPoint>();
	}

	@Override
	public VisObject getVisObject() {
		VisChain c = new VisChain();
		c.add(new VzOpenBox(scale_xyz, new VzMesh.Style(color)));
		// Uncomment to also draw anchor points
		for(AnchorPoint anchor : anchors){
			c.add(new VisChain(LinAlg.translate(anchor.xyz), 
						new VzBox(new double[]{ 0.04, 0.04, 0.04}, new VzMesh.Style(Color.black))));
		}
		return c;
	}

	public void addObject(RosieSimObject obj){
		for(AnchorPoint pt : anchors){
			// Get the point in world coordinates
			double[][] pt_p = LinAlg.translate(pt.xyz[0], pt.xyz[1], pt.xyz[2] + obj.getScale()[2]/2 + 0.001);
			pt_p = LinAlg.matrixAB(getPose(), pt_p);
			double[] anchorPos = LinAlg.matrixToXyzrpy(pt_p);
			if(pt.object == null || LinAlg.squaredDistance(anchorPos, pt.object.getXYZRPY(), 2) > 0.01){
				pt.object = obj;
				obj.setPose(pt_p);
				return;
			}
		}
		System.err.println("Error adding " + obj.getDescription() + " to " + 
				desc + ", anchors are full");
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		anchors = AnchorPoint.create(scale_xyz[0], scale_xyz[1], -scale_xyz[2]/2, ANCHOR_SPACING);
	}
}
