package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.mobilesim.vis.VzOpenBox;

public class SimLightSwitch extends RosieSimObject {
	private boolean isOn = true;

	private String regionHandle = null;
	private SimRegion region = null;
	
	public SimLightSwitch(SimWorld sw){
		super(sw);
	}

	@Override
	public VisObject getVisObject() {
		return new VzBox(scale_xyz, new VzMesh.Style(isOn ? Color.green : Color.red));
	}

	@Override
	public void performDynamics(ArrayList<SimObject> worldObjects){
		if(region != null){
			return;
		}
		for(SimObject obj : worldObjects){
			if(!(obj instanceof SimRegion)){
				continue;
			}
			SimRegion r = (SimRegion)obj;
			if(r.getHandle().equals(regionHandle)){
				region = (SimRegion)obj;
				region.setLights(isOn);
				return;
			}
		}
	}

	@Override
	public void setState(String property, String value){
		super.setState(property, value);
		if(property.equals("activation1")){
			isOn = value.equals("on2");
			region.setLights(isOn);
		}
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		super.read(ins);
		// [String] Switch State: << on off >>
		isOn = ins.readString().toLowerCase().equals("on");
		properties.put("activation1", isOn ? "on2" : "off2");

		// [String] regionHandle
		regionHandle = ins.readString();
	}

	@Override
	public void write(StructureWriter outs) throws IOException
	{
		super.write(outs);
		outs.writeString(isOn ? "on" : "off");
		outs.writeString(regionHandle);
	}
}
