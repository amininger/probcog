package probcog.sim;

import java.io.IOException;

import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisObject;
import april.vis.VzBox;
import april.vis.VzMesh;

public class SimBoxPC extends SimObjectPC{
	protected double[] lwh = new double[]{1, 1, 1};

	public SimBoxPC(SimWorld sw) {
		super(sw);
	}

	@Override
	public Shape getShape() {
		return new BoxShape(scale * lwh[0], scale * lwh[1], scale * lwh[2]);
	}

	@Override
	public VisObject getVisObject() {
		return new VzBox(scale * lwh[0], scale * lwh[1], scale * lwh[2], new VzMesh.Style(color));
	}
	
	/** Restore state that was previously written **/
	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		
		// 3 doubles for size (lwh)
		lwh = ins.readDoubles();
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
	@Override
    public void write(StructureWriter outs) throws IOException
    {
		super.write(outs);
		outs.writeDoubles(lwh);
    }
}
