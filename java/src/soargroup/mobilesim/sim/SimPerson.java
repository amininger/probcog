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
	protected Color shirtColor;

	public SimPerson(SimWorld sw){
		super(sw);
	}

	@Override
	public VisChain createVisObject() {
		VisChain vc = new VisChain();

		double head_r = scale_xyz[0]*0.8;
		double shoulder_h = scale_xyz[2]/2 - head_r*2;
		double torso_h = (scale_xyz[2] - head_r*2)*0.5;
		double legs_h = (scale_xyz[2] - head_r*2 - torso_h);
		double limb_w = 0.12;

		this.anchors.add(new AnchorPoint(limb_w, -scale_xyz[1]/2+limb_w/2, shoulder_h - torso_h, this, "on"));

		Color legColor = new Color(50, 20, 20);

		vc.add(new VisChain(LinAlg.translate(0.0, 0.0, scale_xyz[2]/2 - head_r), 
					new VzSphere(head_r, new VzMesh.Style(new Color(224, 172, 105)))));

		vc.add(new VisChain(LinAlg.translate(0.0, 0.0, shoulder_h - torso_h/2), 
					new VzBox(scale_xyz[0], scale_xyz[1]-2*limb_w, torso_h, new VzMesh.Style(shirtColor))));

		vc.add(new VisChain(LinAlg.translate(0.0, scale_xyz[1]/2-limb_w/2, shoulder_h - limb_w - torso_h/2), 
					new VzBox(limb_w, limb_w, torso_h, new VzMesh.Style(shirtColor))));

		vc.add(new VisChain(LinAlg.translate(0.0, -scale_xyz[1]/2+limb_w/2, shoulder_h - limb_w - torso_h/2), 
					new VzBox(limb_w, limb_w, torso_h, new VzMesh.Style(shirtColor))));

		vc.add(new VisChain(LinAlg.translate(0.0, -limb_w*0.6, -scale_xyz[2]/2 + legs_h/2),
					new VzBox(limb_w, limb_w, legs_h, new VzMesh.Style(legColor))));

		vc.add(new VisChain(LinAlg.translate(0.0, limb_w*0.6, -scale_xyz[2]/2 + legs_h/2),
					new VzBox(limb_w, limb_w, legs_h, new VzMesh.Style(legColor))));

		return vc;
	}

    /** Restore state that was previously written **/
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);

		// Default size
		scale_xyz[0] *= 0.25;
		scale_xyz[1] *= 0.5;
		scale_xyz[2] *= 1.8;

		// [Int]x3 shirt color
    	int rgb[] = ins.readInts();
    	shirtColor = new Color(rgb[0], rgb[1], rgb[2]);
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
    public void write(StructureWriter outs) throws IOException
    {
		super.write(outs);
    	int rgb[] = new int[]{ shirtColor.getRed(), shirtColor.getGreen(), shirtColor.getBlue() };
    	outs.writeInts(rgb);
	}
}
