package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.StructureReader;
import april.util.StructureWriter;

public class SimBoxObject extends RosieSimObject {
	protected Color  color = Color.gray;

	public SimBoxObject(SimWorld sw){
		super(sw);
	}

	@Override
	public VisObject getVisObject() {
		// This is a wireframe box
		//	return new VzBox(scale_xyz, new VzLines.Style(color, 1));
		return new VzBox(scale_xyz, new VzMesh.Style(color));
	}

	@Override
	public void performDynamics(ArrayList<RosieSimObject> worldObjects){ 
		// No dynamics
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);

    	// [Int]x3 rgb color
    	int rgb[] = ins.readInts();
    	color = new Color(rgb[0], rgb[1], rgb[2]);
	}

	@Override
    public void write(StructureWriter outs) throws IOException
    {
		super.write(outs);

    	int rgb[] = new int[]{ color.getRed(), color.getGreen(), color.getBlue() };
    	outs.writeInts(rgb);
    }

}
