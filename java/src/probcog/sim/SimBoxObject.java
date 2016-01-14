package probcog.sim;

import april.sim.*;
import april.vis.*;

public class SimBoxObject extends SimObjectPC{
	public SimBoxObject(SimWorld sw){
		super(sw);
	}

	@Override
	public VisObject getVisObject() {
		return new VzBox(scale_xyz, new VzMesh.Style(color));
	}

	@Override
	public Shape getShape() {
		return new BoxShape(scale_xyz);
	}
}
