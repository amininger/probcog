package probcog.sim;

import java.io.IOException;

import april.sim.BoxShape;
import april.sim.Shape;
import april.sim.SimWorld;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.*;
import april.jmat.LinAlg;

public class SimBoxPC extends SimObjectPC{
	public SimBoxPC(SimWorld sw) {
		super(sw);
	}

	@Override
	public VisObject getVisObject() {
		return new VisChain(
				LinAlg.scale(scale),
				new VzBox(lenxyz[0], lenxyz[1], lenxyz[2], new VzMesh.Style(color))
		);
	}
	
	/** Restore state that was previously written **/
	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		
		// 3 doubles for size (lwh)
		lenxyz = ins.readDoubles();
    }

    /** Write one or more lines that serialize this instance. No line
     * is allowed to consist of just an asterisk. **/
	@Override
    public void write(StructureWriter outs) throws IOException
    {
		super.write(outs);
		outs.writeDoubles(lenxyz);
    }
}
