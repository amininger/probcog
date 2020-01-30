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

public class SimLightSwitch extends RosieSimObject {
	private boolean isOn = true;

	private String regionHandle = null;
	private SimRegion region = null;
	
	public SimLightSwitch(SimWorld sw){
		super(sw);
	}

	@Override
	public VisChain createVisObject() {
		return new VisChain(
			new VzBox(scale_xyz, new VzMesh.Style(isOn ? Color.green : Color.red))
		);
	}

	@Override
	public void setup(ArrayList<SimObject> worldObjects){
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
		if(region == null){
			System.err.println("ERROR: LightSwitch cannot find region " + regionHandle);
		}
	}

	@Override
	public void setState(String property, String value){
		super.setState(property, value);
		if(property.equals(RosieConstants.ACTIVATION)){
			isOn = value.equals(RosieConstants.ACT_ON);
			region.setLights(isOn);
		}
	}

	@Override
    public void read(StructureReader ins) throws IOException
    {
		// [String] regionHandle
		regionHandle = ins.readString();

		super.read(ins);
		isOn = properties.get(RosieConstants.ACTIVATION).equals(RosieConstants.ACT_ON);
	}

	@Override
	public void write(StructureWriter outs) throws IOException
	{
		outs.writeString(regionHandle);
		super.write(outs);
	}
}
