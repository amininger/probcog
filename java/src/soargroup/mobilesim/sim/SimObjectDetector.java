package soargroup.mobilesim.sim;

import java.io.IOException;
import java.util.*;

import java.nio.ByteBuffer;

import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.PeriodicTasks;
import april.util.TimeUtil;

import soargroup.mobilesim.sim.SimRobot;
import soargroup.mobilesim.sim.RosieSimObject;
import soargroup.mobilesim.sim.SimRegion;
import soargroup.mobilesim.util.BoundingBox;
import soargroup.mobilesim.util.Util;

// LCM Types
import lcm.lcm.*;
import april.lcmtypes.pose_t;
import soargroup.mobilesim.lcmtypes.object_data_t;
import soargroup.mobilesim.lcmtypes.object_data_list_t;


public class SimObjectDetector {
	private static double MSG_PER_SEC = 10.0;

	protected SimRobot robot;
	protected SimWorld world;

	protected HashSet<RosieSimObject> detectedObjects;
	protected List<SimRegion> regions = null;

    static Random classifierRandom = new Random(3611871);

    PeriodicTasks tasks = new PeriodicTasks(2);

	public SimObjectDetector(SimRobot robot, SimWorld world){
		this.robot = robot;
		this.world = world;
		this.detectedObjects = new HashSet<RosieSimObject>();

		this.tasks.addFixedDelay(new DetectorTask(), 1.0/MSG_PER_SEC);

	}

	public void setRunning(boolean b){
		tasks.setRunning(b);
	}

	protected class DetectorTask implements PeriodicTasks.Task {
        public void run(double dt){
            ArrayList<SimObject> simObjects;
            synchronized(world.objects){
            	simObjects = (ArrayList<SimObject>)world.objects.clone();
				if(regions == null){
					regions = new ArrayList<SimRegion>();
					for(SimObject obj : world.objects){
						if(obj instanceof SimRegion){
							regions.add((SimRegion)obj);
						}
					}
				}
            }
			

			updateDetectedObjects(simObjects);
			sendObjectMessage();
        }

        private void updateDetectedObjects(ArrayList<SimObject> simObjects){
			HashSet<RosieSimObject> rosieObjects = new HashSet<RosieSimObject>();
			for(SimObject obj : simObjects){
				if(obj instanceof RosieSimObject){
					rosieObjects.add((RosieSimObject)obj);
				}
			}

			synchronized(detectedObjects){
				detectedObjects.clear();
				for(RosieSimObject obj : rosieObjects){
					SimRegion robotRegion = robot.getRegion();
					if (obj == robot.getGrabbedObject() || (obj.getRegion(regions) == robotRegion &&
						robot.inViewRange(obj.getBoundingBox().xyzrpy))){
						detectedObjects.add(obj);
					}
				}
			}
		}

        private void sendObjectMessage(){
			object_data_list_t objectMessage = new object_data_list_t();
			objectMessage.utime = TimeUtil.utime();
			synchronized(detectedObjects){
				objectMessage.num_objects = 0;
				objectMessage.objects = new object_data_t[detectedObjects.size()];
				for(RosieSimObject obj : detectedObjects){
					objectMessage.objects[objectMessage.num_objects] = obj.getObjectData();
					objectMessage.num_objects += 1;
				}
			}
            LCM.getSingleton().publish("DETECTED_OBJECTS", objectMessage);
        }
    }
}
