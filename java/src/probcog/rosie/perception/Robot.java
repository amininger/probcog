package probcog.rosie.perception;

import probcog.lcmtypes.tag_classification_list_t;
import probcog.lcmtypes.tag_classification_t;
import sml.Identifier;
import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.SVSCommands;
import edu.umich.rosie.soar.SoarUtil;
import edu.umich.rosie.soar.StringWME;
import edu.umich.rosie.soarobjects.Pose;

public class Robot implements ISoarObject {
	private static final double VIEW_DIST = 3.8;
	private static final double VIEW_ANGLE = Math.PI/2 * .8;
	private static final double VIEW_HEIGHT = 2.0;
	
	private static final double[] dims = new double[]{.5, .5, .5};
	
	private double[] pos = new double[]{ 0.0, 0.0, 0.0 };
	private double[] rot = new double[]{ 0.0, 0.0, 0.0 };
	private Pose pose;
	
	private boolean needsUpdate = true;
	
	private StringBuilder svsCommands = new StringBuilder();
	
	private Identifier selfId = null;
	
	private StringWME movingState;
	
	private tag_classification_list_t curTagClassifications = null;
	private boolean newClassifications = false;

	private Identifier waypointId = null;
	private int curWaypoint = -1;
	
	public Robot(){
		movingState = new StringWME("moving-state", "stopped");
		pose = new Pose();
	}
	
	public void updatePose(double[] xyzrpy){
		for(int d = 0; d < 3; d++){
			if(Math.abs(pos[d] - xyzrpy[d]) > 0.02){
				pos[d] = xyzrpy[d];
				needsUpdate = true;
			}
		}
		for(int d = 0; d < 0; d++){
			if(Math.abs(rot[d] - xyzrpy[3+d]) > 0.05){
				rot[d] = xyzrpy[3+d];
				needsUpdate = true;
			}
		}
		if(needsUpdate){
			pose.updateWithArray(xyzrpy);
		}
	}
	
	 public void updateClassifications(tag_classification_list_t newTagClassifications){
		 curTagClassifications = newTagClassifications;
		 newClassifications = true;
	 }
	 
	 public void updateMovingState(String newMovingState){
		 movingState.setValue(newMovingState);
	 }
	
	/* Creates a triangular view region of height VIEW_DIST
	 * and angle VIEW_ANGLE and a height of 2m
	 */
	public static String getViewRegionVertices(){
		StringBuilder verts = new StringBuilder();
		double dx = VIEW_DIST/2;
		double dy = VIEW_DIST * Math.sin(VIEW_ANGLE/2);
		double dz = VIEW_HEIGHT/2;
		// Top triangle
		verts.append(String.format("%f %f %f ", -dx, 0.0, dz));
		verts.append(String.format("%f %f %f ", dx, -dy, dz));
		verts.append(String.format("%f %f %f ", dx, dy, dz));
		// Top triangle
		verts.append(String.format("%f %f %f ", -dx, 0.0, -dz));
		verts.append(String.format("%f %f %f ", dx, -dy, -dz));
		verts.append(String.format("%f %f %f", dx, dy, -dz));
		
		return verts.toString();
	}
	
	public String getSVSCommands(){
		String cmds = svsCommands.toString();
		svsCommands = new StringBuilder();
		return cmds;
	}
	
	private boolean added = false;

	public boolean isAdded() {
		return added;
	}

	@Override
	public void addToWM(Identifier parentId) {
		if(added){
			return;
		}
		selfId = parentId.CreateIdWME("self");
		pose.addToWM(selfId);
		movingState.addToWM(selfId);
		
		svsCommands.append(String.format("add robot world p %s r %s\n", 
				SVSCommands.posToStr(pos), SVSCommands.rotToStr(rot)));
		svsCommands.append(String.format("add robot_pos robot\n"));
		svsCommands.append(String.format("add robot_body robot v %s p .2 0 0 s %s\n", 
				SVSCommands.bboxVertices(), SVSCommands.scaleToStr(dims)));
		svsCommands.append(String.format("add robot_view robot v %s p %f %f %f\n", 
				getViewRegionVertices(), VIEW_DIST/2 + .5, 0.0, VIEW_HEIGHT/2 - dims[2]/2));

		added = true;
	}

	@Override
	public void updateWM() {
		if(!added){
			return;
		}
		pose.updateWM();
		movingState.updateWM();
		if(needsUpdate){
			svsCommands.append(SVSCommands.changePos("robot", pos));
			svsCommands.append(SVSCommands.changeRot("robot", rot));
			needsUpdate = false;
		}
		if(newClassifications){
			updateWaypointInfo();
			newClassifications = false;
		}
	}
	
	private void updateWaypointInfo(){
    	int closestWaypoint = -1;
    	double closestDistance = Double.MAX_VALUE;
    	for (tag_classification_t c : curTagClassifications.classifications){
    		if (c.range < closestDistance){
    			closestWaypoint = c.tag_id;
    			closestDistance = c.range;
    		}
    	}
   		if (closestWaypoint != curWaypoint && waypointId != null){
   			waypointId.DestroyWME();
   			waypointId = null;
   		}
   		curWaypoint = closestWaypoint;
   		if (curWaypoint == -1){
   			return;
   		}
   		if (waypointId == null){
    		waypointId = selfId.CreateIdWME("current-waypoint");
   		}
   		for (tag_classification_t c : curTagClassifications.classifications){
   			if (c.tag_id == closestWaypoint){
   				if (c.name.startsWith("wp")){
   					SoarUtil.updateStringWME(waypointId, "waypoint-handle", c.name);
   				} else {
   					SoarUtil.updateStringWME(waypointId, "classification", c.name);
   				}
   			}
   		}
	}

	@Override
	public void removeFromWM() {
		if(!added){
			return;
		}
		
		if(waypointId != null){
			waypointId.DestroyWME();
			waypointId = null;
		}
		movingState.removeFromWM();
		pose.removeFromWM();

		selfId.DestroyWME();
		selfId = null;
		
		svsCommands.append(String.format("delete robot\n"));
		added = false;
	}

}
