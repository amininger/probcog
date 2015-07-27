package probcog.rosie.perception;

import java.io.IOException;
import java.util.Properties;

import javax.swing.JMenuBar;

import edu.umich.rosie.soar.AgentConnector;
import edu.umich.rosie.soar.SoarAgent;
import edu.umich.rosie.soar.SoarUtil;
import edu.umich.rosie.soarobjects.Pose;
import april.lcmtypes.pose_t;
import lcm.lcm.LCM;
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMSubscriber;
import probcog.lcmtypes.classification_list_t;
import probcog.lcmtypes.classification_t;
import probcog.rosie.actuation.MobileActuationConnector;
import sml.Identifier;

public class MobilePerceptionConnector extends AgentConnector implements LCMSubscriber{
	private boolean newClassifications = false;
	private classification_list_t curClassifications = null;
	private Identifier waypointId = null;
	private int curWaypoint = -1;

    private LCM lcm;
    
    private Identifier selfId = null;
    
    private boolean newPose = false;
    private Pose pose;

    public MobilePerceptionConnector(SoarAgent agent, Properties props){
    	super(agent);
    	
    	pose = new Pose();

    	// Setup LCM events
        lcm = LCM.getSingleton();
    }
    
    @Override
    public void connect(){
    	super.connect();
        lcm.subscribe("CLASSIFICATIONS.*", this);
        lcm.subscribe("POSE", this);
    }
    
    @Override
    public void disconnect(){
    	super.disconnect();
        lcm.unsubscribe("CLASSIFICATIONS.*", this);
        lcm.unsubscribe("POSE", this);
    }

	@Override
	public void createMenu(JMenuBar menuBar) {}

    @Override
    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins){
		try {
			if(channel.startsWith("CLASSIFICATIONS")){
				curClassifications = new classification_list_t(ins);
				newClassifications = true;
			} else if (channel.startsWith("POSE")){
				pose_t p = new pose_t(ins);
				pose.updatePosition(p.pos);
				newPose = true;
			}
		} catch (IOException e) {
			e.printStackTrace();
		}
    }
    
    protected synchronized void onInputPhase(Identifier inputLink){
    	if(selfId == null){
    		selfId = inputLink.CreateIdWME("self");
    	}
    	updateSelf();
    	updateClassifications();
    }
    
    private void updateSelf(){
    	if(!pose.isAdded()){
    		pose.addToWM(selfId);
    	} else if(newPose){
    		pose.updateWM();
    		newPose = false;
    	}

    	String movingState = ((MobileActuationConnector)soarAgent.getActuationConnector()).getMovingState();
		SoarUtil.updateStringWME(selfId, "moving-state", movingState);
    }
    
    private void updateClassifications(){
    	if(!newClassifications){
    		return;
    	}
    	newClassifications = false;
    	
    	int closestWaypoint = -1;
    	double closestDistance = Double.MAX_VALUE;
    	for (classification_t c : curClassifications.classifications){
    		if (c.range < closestDistance){
    			closestWaypoint = c.id;
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
   		for (classification_t c : curClassifications.classifications){
   			if (c.id == closestWaypoint){
   				if (c.name.startsWith("wp")){
   					SoarUtil.updateStringWME(waypointId, "waypoint-handle", c.name);
   				} else {
   					SoarUtil.updateStringWME(waypointId, "classification", c.name);
   				}
   			}
   		}
    }


	@Override
	protected void onInitSoar() {
		pose.removeFromWM();
		if(selfId != null){
			selfId.DestroyWME();
			selfId = null;
			waypointId = null;
		}
	}
    
    // Currently no commands relevant to perception
	protected void onOutputEvent(String attName, Identifier id) { }
}
