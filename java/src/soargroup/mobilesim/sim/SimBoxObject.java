package soargroup.mobilesim.sim;

import april.sim.*;
import april.vis.*;

public class SimBoxObject extends SimObjectPC{
	public SimBoxObject(SimWorld sw){
		super(sw);
	}

	@Override
	public VisObject getVisObject() {
		if(color.getRed() == 0 && color.getGreen() == 0 && color.getBlue() == 0){
			return new VzBox(scale_xyz, new VzLines.Style(color, 1));
		} else {
			return new VzBox(scale_xyz, new VzMesh.Style(color));
		}
	}

	@Override
	public Shape getShape() {
		return new SphereShape(-.5);
	}
}
