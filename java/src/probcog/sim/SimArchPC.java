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
import april.vis.VzRectangle;

public class SimArchPC extends SimObjectPC{

	public SimArchPC(SimWorld sw) {
		super(sw);
	}

	@Override
	public Shape getShape() {
	    Shape leg1 = new BoxShape(LinAlg.scale(new double[]{1, 1, 1}, scale));
	    Shape leg2 = new BoxShape(LinAlg.scale(new double[]{1, 1, 1}, scale));
	    Shape top = new BoxShape(LinAlg.scale(new double[]{3, 1, 1}, scale));
	    return new CompoundShape(LinAlg.translate(new double[]{scale, -scale/2, 0}), leg1,
	                LinAlg.translate(new double[]{-2*scale, 0, 0}), leg2,
	                LinAlg.translate(new double[]{scale, scale, 0}), top);
	}

	@Override
	public VisObject getVisObject() {
		ArrayList<Object> objs = new ArrayList<Object>();

        // The right leg of the arch
        objs.add(new VisChain(LinAlg.scale(scale), LinAlg.translate(1, -.5, 0), 
        		new VzBox(new VzMesh.Style(color))));
        
        // The left leg of the arch
        objs.add(new VisChain(LinAlg.scale(scale), LinAlg.translate(-1, -.5, 0), 
        		new VzBox(new VzMesh.Style(color))));
        
        // The larger box making up the top
        objs.add(new VisChain(LinAlg.scale(scale), LinAlg.translate(0, .5, 0), 
        		LinAlg.scale(3, 1, 1), new VzBox(new VzMesh.Style(color))));

        return new VisChain(objs.toArray());
	}
	


}
