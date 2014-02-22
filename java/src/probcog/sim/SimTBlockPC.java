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

public class SimTBlockPC extends SimObjectPC{

	public SimTBlockPC(SimWorld sw) {
		super(sw);
	}

	@Override
	public Shape getShape() {
	    Shape top = new BoxShape(LinAlg.scale(new double[]{3, 1, 1}, scale));
	    Shape bottom = new BoxShape(LinAlg.scale(new double[]{1, 1, 1}, scale));
	    return new CompoundShape(LinAlg.translate(new double[]{0, scale/2, 0}), top,
	                LinAlg.translate(new double[]{0, -scale, 0}), bottom);
	}

	@Override
	public VisObject getVisObject() {
		ArrayList<Object> objs = new ArrayList<Object>();

        // The top of the T 
        objs.add(new VisChain(LinAlg.scale(scale), LinAlg.translate(0, .5, 0), 
        		LinAlg.scale(3, 1, 1), new VzBox(new VzMesh.Style(color))));
        
        // The bottom of the T
        objs.add(new VisChain(LinAlg.scale(scale), LinAlg.translate(0, -.5, 0), 
        		new VzBox(new VzMesh.Style(color))));
        

        return new VisChain(objs.toArray());
	}
}
