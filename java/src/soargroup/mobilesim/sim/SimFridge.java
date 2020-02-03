package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;
import soargroup.rosie.RosieConstants;

public class SimFridge extends SimShelves {
	public static final double TEMPERATURE = 35.0;

	public SimFridge(SimWorld sw){
		super(sw);
	}

	// Children can override to implement any dynamics, this is called multiple times/second
	public void performDynamics(ArrayList<SimObject> worldObjects) {
		//for(AnchorPoint pt : anchors){
		//	if(!pt.hasObject()){
		//		continue;
		//	}
		//	pt.checkObject();
		//	if(pt.object != null){
		//		pt.object.changeTemperature(TEMPERATURE);
		//	}
		//}
	}
}
