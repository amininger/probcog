package soargroup.mobilesim.sim;

import java.awt.Color;
import java.util.ArrayList;
import java.io.IOException;

import april.sim.*;
import april.vis.*;
import april.util.*;
import april.jmat.LinAlg;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.util.ResultTypes.*;

import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.attributes.*;

public class SimAlarm extends RosieSimObject {

	protected Activatable activatable;
	protected boolean isOn = false;
	protected long onTime = 0;
	private final static long DURATION = 100; // seconds before it automatically turns off again
	
	public SimAlarm(SimWorld sw){
		super(sw);
	}

	@Override
	public void init(ArrayList<SimObject> worldObjects){
		properties.put(RosieConstants.ACTIVATION, RosieConstants.ACT_OFF);
		activatable = new Activatable(this, false);
		addAttribute(activatable);
	}

	@Override
	public void update(double dt) { 
		super.update(dt);
		if(!isOn && activatable.isOn()){
			// Just turned on, start the timer and turn green
			isOn = true;
			onTime = TimeUtil.utime();
			color = Color.green;
			recreateVisObject();
			return;
		}
		long elapsed = TimeUtil.utime() - onTime;
		if(isOn && (!activatable.isOn() || elapsed > DURATION)){
			// Either explicitly turned off, or the timer elapsed
			isOn = false;
			color = Color.red;
			recreateVisObject();
		}
	}
}
