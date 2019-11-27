package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;

public class SimPerson extends RosieSimObject {
	protected AnchorPoint anchor;

	protected Color shirtColor;

	protected VisObject cachedVisObject;

	public SimPerson(SimWorld sw){
		super(sw);
	}

	@Override
	public VisObject getVisObject() {
		if(cachedVisObject == null){
			VisChain vc = new VisChain();

			double head_r = scale_xyz[0]*0.8;
			double shoulder_h = scale_xyz[2]/2 - head_r*2;
			double torso_h = (scale_xyz[2] - head_r*2)*0.5;
			double legs_h = (scale_xyz[2] - head_r*2 - torso_h);
			double limb_w = 0.1;

			Color legColor = new Color(50, 20, 20);

			vc.add(new VisChain(LinAlg.translate(0.0, 0.0, scale_xyz[2]/2 - head_r), 
						new VzSphere(head_r, new VzMesh.Style(new Color(224, 172, 105)))));

			vc.add(new VisChain(LinAlg.translate(0.0, 0.0, shoulder_h - torso_h/2), 
						new VzBox(scale_xyz[0], scale_xyz[1]-2*limb_w, torso_h, new VzMesh.Style(shirtColor))));

			vc.add(new VisChain(LinAlg.translate(0.0, scale_xyz[1]/2-limb_w/2, shoulder_h - limb_w - torso_h/2), 
						new VzBox(limb_w, limb_w, torso_h, new VzMesh.Style(shirtColor))));

			vc.add(new VisChain(LinAlg.translate(0.0, -scale_xyz[1]/2+limb_w/2, shoulder_h - limb_w - torso_h/2), 
						new VzBox(limb_w, limb_w, torso_h, new VzMesh.Style(shirtColor))));

			vc.add(new VisChain(LinAlg.translate(0.0, -limb_w, -scale_xyz[2]/2 + legs_h/2),
						new VzBox(limb_w, limb_w, legs_h, new VzMesh.Style(legColor))));

			vc.add(new VisChain(LinAlg.translate(0.0, limb_w, -scale_xyz[2]/2 + legs_h/2),
						new VzBox(limb_w, limb_w, legs_h, new VzMesh.Style(legColor))));

			cachedVisObject = vc;
		}
		return cachedVisObject;
	}

	public void performDynamics(ArrayList<SimObject> worldObjects){}

	public void addObject(RosieSimObject obj){
		// Get the point in world coordinates
		double[][] pt_p = LinAlg.translate(anchor.xyz[0], anchor.xyz[1], anchor.xyz[2] + obj.getScale()[2]/2 + 0.001);
		pt_p = LinAlg.matrixAB(getPose(), pt_p);
		double[] anchorPos = LinAlg.matrixToXyzrpy(pt_p);
		if(anchor.object == null || LinAlg.squaredDistance(anchorPos, anchor.object.getXYZRPY(), 2) > 0.01){
			anchor.object = obj;
			obj.setPose(pt_p);
			return;
		}
		System.err.println("Error adding " + obj.getDescription() + " to " + 
				desc + ", already holding something");
	}

    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
        // [Str] description
        desc = ins.readString();

    	// [Dbl]x3 xyz center of object
    	double[] xyz = ins.readDoubles();

		// [Dbl] rotation (xy plane)
		double yaw = ins.readDouble();
		xyzrpy = new double[]{ xyz[0], xyz[1], xyz[2], 0.0, 0.0, yaw };

    	scale_xyz = new double[]{ 0.25, 0.5, 1.8 };

		// [String] category
		properties.put("category", ins.readString());

		// [String] name
		properties.put("name", ins.readString());

		// [Int]x3 shirt color
    	int rgb[] = ins.readInts();
    	shirtColor = new Color(rgb[0], rgb[1], rgb[2]);
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
		outs.writeString(desc);
    	outs.writeDoubles(LinAlg.copy(xyzrpy, 3));
		outs.writeDouble(xyzrpy[5]);
		outs.writeString(properties.get("category"));
		outs.writeString(properties.get("name"));
    	int rgb[] = new int[]{ shirtColor.getRed(), shirtColor.getGreen(), shirtColor.getBlue() };
    	outs.writeInts(rgb);
	}
}
