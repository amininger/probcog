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
		this.lenxyz = new double[]{3, 2, 1};
	}

	@Override
	public Shape getShape() {
	    return new CompoundShape(
	    		LinAlg.scale(scale), 
	    		LinAlg.translate(1, -0.5, 0), new BoxShape(1, 1, 1), // Left Leg
	    		LinAlg.translate(-2, 0, 0), new BoxShape(1, 1, 1),   // Right Leg
	    		LinAlg.translate(1, 1, 0), new BoxShape(3, 1, 1)     // Top Bar
	    );
	}

	@Override
	public VisObject getVisObject() {
		ArrayList<Object> objs = new ArrayList<Object>();
		objs.add(LinAlg.scale(scale));

        // The right leg of the arch
        objs.add(new VisChain(LinAlg.translate(1, -.5, 0), 
        		new VzBox(new VzMesh.Style(color))));
        
        // The left leg of the arch
        objs.add(new VisChain(LinAlg.translate(-1, -.5, 0), 
        		new VzBox(new VzMesh.Style(color))));
        
        // The larger box making up the top
        objs.add(new VisChain(LinAlg.translate(0, .5, 0), 
        		LinAlg.scale(3, 1, 1), new VzBox(new VzMesh.Style(color))));

        return new VisChain(objs.toArray());
	}
}
