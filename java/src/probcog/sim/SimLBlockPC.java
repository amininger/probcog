package probcog.sim;

import java.util.ArrayList;

import april.jmat.LinAlg;
import april.sim.BoxShape;
import april.sim.CompoundShape;
import april.sim.Shape;
import april.sim.SimWorld;
import april.vis.VisChain;
import april.vis.VisObject;
import april.vis.VzBox;
import april.vis.VzMesh;

public class SimLBlockPC extends SimObjectPC{

	public SimLBlockPC(SimWorld sw) {
		super(sw);
	}

	@Override
	public Shape getShape() {
	    Shape side = new BoxShape(LinAlg.scale(new double[]{1, 3, 1}, scale));
	    Shape bottom = new BoxShape(LinAlg.scale(new double[]{1, 1, 1}, scale));
	    return new CompoundShape(LinAlg.translate(new double[]{-scale/2, 0, 0}), side,
	                LinAlg.translate(new double[]{scale, -scale, 0}), bottom);
	}

	@Override
	public VisObject getVisObject() {
		ArrayList<Object> objs = new ArrayList<Object>();

        // The side of the L
        objs.add(new VisChain(LinAlg.scale(scale), LinAlg.translate(-.5, 0, 0), 
        		LinAlg.scale(1, 3, 1), new VzBox(new VzMesh.Style(color))));
        
        // The bottom of the L
        objs.add(new VisChain(LinAlg.scale(scale), LinAlg.translate(.5, -1, 0), 
        		new VzBox(new VzMesh.Style(color))));
        

        return new VisChain(objs.toArray());
	}
}
