package soargroup.mobilesim.sim.attributes;

import java.util.ArrayList;

import soargroup.rosie.RosieConstants;
import soargroup.mobilesim.util.ResultTypes.*;

import soargroup.mobilesim.sim.*;
import soargroup.mobilesim.sim.actions.*;
import soargroup.mobilesim.sim.actions.ActionHandler.*;

public class Fillable extends Attribute {
	private String contents;

	public Fillable(RosieSimObject baseObject, String contents){
		super(baseObject);
		this.contents = contents;
	}

	public String getContents(){
		return contents;
	}

	public void setContents(String contents){
		this.contents = contents;
		baseObject.setProperty(RosieConstants.CONTENTS, contents);
		baseObject.recreateVisObject();
	}
}
