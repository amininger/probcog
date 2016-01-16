package probcog.perception;

import java.io.IOException;
import java.util.ArrayList;
import java.util.Collection;
import java.util.HashMap;
import java.util.Map;
import java.util.Random;

import lcm.lcm.LCM;
import magic2.lcmtypes.tag_detection_list_t;
import magic2.lcmtypes.tag_detection_t;
import april.jmat.LinAlg;
import april.jmat.MathUtil;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.PeriodicTasks;
import april.util.TimeUtil;
import probcog.classify.TagClassifier;
import probcog.classify.TagHistory;
import probcog.lcmtypes.classification_list_t;
import probcog.lcmtypes.classification_t;
import probcog.lcmtypes.object_data_t;
import probcog.lcmtypes.soar_objects_t;
import probcog.lcmtypes.tag_classification_list_t;
import probcog.lcmtypes.tag_classification_t;
import probcog.old.sim.SimDoor;
import probcog.sim.SimAprilTag;
import probcog.sim.SimFalseDoor;
import probcog.sim.SimHallway;
import probcog.sim.SimObjectPC;
import probcog.sim.SimRobot;
import probcog.util.BoundingBox;
import probcog.util.Util;

public class SimObjectDetector {
	private static double MSG_PER_SEC = 10.0;
	
	protected SimRobot robot;
	protected SimWorld world;
	
	protected HashMap<SimObjectPC, Integer> detectedObjects;
	private Integer nextID = 1;

    TagHistory tagHistory = new TagHistory();
    static Random classifierRandom = new Random(3611871);
	
    PeriodicTasks tasks = new PeriodicTasks(2);
	
	public SimObjectDetector(SimRobot robot, SimWorld world){
		this.robot = robot;
		this.world = world;
		this.detectedObjects = new HashMap<SimObjectPC, Integer>();
		
		tasks.addFixedDelay(new DetectorTask(), 1.0/MSG_PER_SEC);
	}
	
	public void setRunning(boolean b){
		tasks.setRunning(b);
	}
	
	protected class DetectorTask implements PeriodicTasks.Task {
        TagClassifier tc;

        public DetectorTask()
        {
            try {
                tc = new TagClassifier(false);
            } catch (IOException ioex) {
                System.err.println("ERR: Could not create TagClassifier");
                ioex.printStackTrace();
            }
        }
        
        public void run(double dt){
            ArrayList<SimObject> simObjects;
            synchronized(world.objects){
            	simObjects = (ArrayList<SimObject>)world.objects.clone();
            }
            
            sendObjectMessage(simObjects);
            detectAprilTags(simObjects, LinAlg.matrixToXyzrpy(robot.getPose()));
        }
        
        private void sendObjectMessage(ArrayList<SimObject> simObjects){
            for(SimObject so : simObjects){
            	if (so instanceof SimObjectPC){
            		// Run through each SimObjectPC for a possible inclusion in the object list
            		SimObjectPC obj = (SimObjectPC)so;
            		// Check if the object is visible by the robot
            		if (robot.inViewRange(obj.getBoundingBox().xyzrpy)){
            			// A newly seen object (not seen in previous frame), give a new id and add to list
            			if(!detectedObjects.containsKey(obj)){
            				detectedObjects.put(obj, nextID);
            				nextID++;
            			}
            		} else {
            			if (detectedObjects.containsKey(obj)){
            				detectedObjects.remove(obj);
            			}
            		}
            	}
            }
            
            soar_objects_t objects = new soar_objects_t();
            objects.utime = TimeUtil.utime();
            objects.num_objects = detectedObjects.size();
            objects.objects = new object_data_t[objects.num_objects];
            int obj_index = 0;
            for(Map.Entry<SimObjectPC, Integer> e : detectedObjects.entrySet()){
            	object_data_t objData = new object_data_t();
            	objData.utime = TimeUtil.utime();
            	objData.id = e.getValue().toString();
            	
            	BoundingBox bbox = e.getKey().getBoundingBox();
            	objData.xyzrpy = bbox.xyzrpy;
            	objData.lenxyz = bbox.lenxyz;
            	
            	HashMap<String, String> classifications = e.getKey().getClassifications();
            	objData.num_classifications = classifications.size();
            	objData.classifications = new classification_t[objData.num_classifications];
            	int cl_index = 0;
            	for(Map.Entry<String, String> cl : classifications.entrySet()){
            		classification_t c = new classification_t();
            		c.category = cl.getKey();
            		c.name = cl.getValue();
            		c.confidence = 1.0;
            		objData.classifications[cl_index++] = c;
            	}
            	
            	objects.objects[obj_index++] = objData;
            }
            LCM.getSingleton().publish("DETECTED_OBJECTS", objects);
        }
        
//		public void run(double dt)
//        {
//            // Look through all objects in the world and if one is
//            // a door and it's within a set distance of us, "classify" it
//            double[] xyzrpyBot = LinAlg.matrixToXyzrpy(robot.getPose());
//            
//            ArrayList<SimObject> simObjects;
//            synchronized(world.objects){
//            	simObjects = (ArrayList<SimObject>)world.objects.clone();
//            }
//
//            for(SimObject so : simObjects) {
//            	if(so instanceof SimHallway){
//            		detectHallway((SimHallway)so, xyzrpyBot);
//            	} else if(so instanceof SimDoor || so instanceof SimFalseDoor){
//            		detectDoor(so, xyzrpyBot);
//            	} else if(so instanceof SimAprilTag){
//            		detectAprilTag((SimAprilTag)so, xyzrpyBot);
//            	}
//            }
//        }
//
//        private classification_t detectDoor(SimObject so, double[] xyzrpyBot)
//        {
//            double sensingThreshold = 1.5;
//
//            double[] xyzrpyDoor = LinAlg.matrixToXyzrpy(so.getPose());
//            double dist = LinAlg.distance(xyzrpyBot, xyzrpyDoor, 2);
//            if (dist > sensingThreshold)
//                return null;
//
//            classification_t classies = new classification_t();
//            classies.utime = TimeUtil.utime();
//            classies.name = "door";
//
//            // Position relative to robot. For now, tossing away orientation data,
//            // but may be relevant later.
//            double[] relXyzrpy = relativePose(robot.getPose(), xyzrpyDoor);
//            classies.xyzrpy = relXyzrpy;
//
//            if (robot.usesNoise()) {
//                // Object detections imperfect. Based on classification confidence
//                if (so instanceof SimDoor) {
//                    SimDoor door = (SimDoor)so;
//                    classies.id = 0; // XXX: door.id;
//                    classies.confidence = 1.0;
//                    // XXX: DOOR: 
//                   // classies.id = door.id;
//                   // classies.confidence = sampleConfidence(door.mean, door.stddev);
//                } else {
//                    SimFalseDoor door = (SimFalseDoor)so;
//                    classies.id = door.id;
//                    classies.confidence = sampleConfidence(door.mean, door.stddev);
//                }
//            } else {
//                // Object detections perfect. Object MUST be a real door
//                if (!(so instanceof SimDoor))
//                    return null;
//                SimDoor door = (SimDoor)so;
//                classies.id = ((SimDoor)so).getID());
//                classies.confidence = 1.0;
//            }
//            return classies;
//        }
//
//        // Only detect hallways that we can "see". That means that
//        // 1) They are not behind the robot
//        // 2) The portal opens in an appropriate direction
//        private classification_t detectHallway(SimHallway hall, double[] xyzrpyBot)
//        {
//            // Imperfect and probably over generous
//            double SENSING_THRESHOLD = 5.0;
//            double ORIENTATION_THRESHOLD = Math.toRadians(-5);
//
//            double[] xyzrpyHall = LinAlg.matrixToXyzrpy(hall.getPose());
//            double[] relXyzrpy = relativePose(LinAlg.xyzrpyToMatrix(xyzrpyBot), xyzrpyHall);
//            double yawHall = xyzrpyHall[5];
//            double yawBot = xyzrpyBot[5];
//            double dotp = Math.cos(yawBot)*Math.cos(yawHall) + Math.sin(yawBot)*Math.sin(yawHall);
//            double dist = LinAlg.distance(xyzrpyBot, xyzrpyHall, 2);
//            // Object must be in range, correctly oriented, and in front of the robot (XXX)
//            if (dist > SENSING_THRESHOLD|| dotp < ORIENTATION_THRESHOLD || relXyzrpy[0] < -0.5)
//                return null;
//
//            classification_t classies = new classification_t();
//            classies.utime = TimeUtil.utime();
//            classies.name = "hallway";
//            classies.xyzrpy = relXyzrpy;
//            classies.id = hall.getID();
//
//            if (robot.usesNoise()) {
//                classies.confidence = sampleConfidence(hall.getMean(), hall.getStdDev());
//            } else {
//                classies.confidence = 1.0;
//            }
//            
//            return classies;
//        }
//
        /** Publish april tag detections. Also directly publishes classifications
         *  right now.
         **/
        private void detectAprilTags(ArrayList<SimObject> sos, double[] xyzrpyBot)
        {
            ArrayList<tag_classification_t> classies = new ArrayList<tag_classification_t>();
            tag_classification_list_t classy_list = new tag_classification_list_t();
            classy_list.utime = TimeUtil.utime();

            ArrayList<tag_detection_t> dets = new ArrayList<tag_detection_t>();
            tag_detection_list_t tdl = new tag_detection_list_t();
            tdl.utime = TimeUtil.utime();

            double iw = Util.getConfig().requireDouble("cameraCalibration.imWidth");
            double ih = Util.getConfig().requireDouble("cameraCalibration.imHeight");

            double cx = iw/2.0;
            double cy = ih/2.0;

            for (SimObject so: sos) {
                if (!(so instanceof SimAprilTag))
                    continue;
                SimAprilTag tag = (SimAprilTag)so;
                double sensingThreshold = 2;

                double[] xyzrpyTag = LinAlg.matrixToXyzrpy(so.getPose());
                double dist = LinAlg.distance(xyzrpyBot, xyzrpyTag, 2);
                if (dist > sensingThreshold)
                    continue;

                // Fake tag detections for 36h11 by a chameleon
                //tag_detection_t td = new tag_detection_t();
                //td.tag_family_bit_width = (byte) 6;
                //td.tag_family_min_hamming_dist = (byte) 11;
                //td.id = tag.getID();
                //td.hamming_dist = 0;
                //td.goodness = 0.0f;


                //dets.add(td);

                // Position relative to robot. For now, tossing away orientation data,
                // but may be relevant later.
                double[] relXyzrpy = relativePose(robot.getPose(), xyzrpyTag);

                long now = TimeUtil.utime();
                ArrayList<tag_classification_t> temp = tc.classifyTag(tag.getID(), relXyzrpy);
                tagHistory.addObservations(temp, now);
                classies.addAll(tagHistory.getLabels(tag.getID(), relXyzrpy, now));
            }
            classy_list.num_classifications = classies.size();
            classy_list.classifications = classies.toArray(new tag_classification_t[0]);
            LCM.getSingleton().publish("CLASSIFICATIONS", classy_list);
        }

        private double sampleConfidence(double u, double s)
        {
            return MathUtil.clamp(u + classifierRandom.nextGaussian()*s, 0, 1);
        }

        private double[] relativePose(double[][] A, double[] xyzrpy)
        {
            double[] xyzrpy_A = LinAlg.matrixToXyzrpy(A);
            double[] p = LinAlg.resize(xyzrpy, 3);
            p = LinAlg.transform(LinAlg.inverse(A), p);
            p = LinAlg.resize(p, 6);

            // Relative yaw difference.
            p[5] = MathUtil.mod2pi(xyzrpy[5] - xyzrpy_A[5]);

            return p;
        }
    }
}
