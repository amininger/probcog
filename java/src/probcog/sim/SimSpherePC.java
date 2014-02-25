package probcog.sim;

import java.awt.Color;
import java.io.IOException;

import april.jmat.LinAlg;
import april.sim.Shape;
import april.sim.SimWorld;
import april.sim.SphereShape;
import april.util.StructureReader;
import april.util.StructureWriter;
import april.vis.VisObject;
import april.vis.VzMesh;
import april.vis.VzSphere;

public class SimSpherePC extends SimObjectPC{

	public SimSpherePC(SimWorld sw) {
		super(sw);
	}

	@Override
	public Shape getShape() {
		return new SphereShape(scale);
	}

	@Override
	public VisObject getVisObject() {
		return new VzSphere(scale, new VzMesh.Style(color));
	}
}
