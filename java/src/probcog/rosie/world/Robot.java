package probcog.rosie.world;

import sml.Agent;
import april.lcmtypes.pose_t;
import april.jmat.LinAlg;
// kinect offset values

/**
 * Representation of the mobile robot positon, rotation, and bounding box in the world
 * Also encodes position of kinect eye
 *
 * @author kirk
 *
 */
public class Robot
{
	// kinect offset values
	final double eyeZ = 0.565;
	final double eyeX = 0.14;
	final double eyeY = 0.00;
    // Name of the object (may be null if not named)
    protected String name;

    // Information about the bounding box
    // Center of the bounding box (XYZ)
    protected double[] bboxPos;

    // Orientation of the bounding box (RPY)
    protected double[] bboxRot;

    // Size of the bounding box (dX, dY, dZ)
    protected double[] bboxSize;

    //offset of kinect camera from centroid
    protected double[] eyeOffset;

    protected double[] centroid;
    protected double[] kinectEye;

    private StringBuilder svsCommands;

    public Robot(){
        name = null;

        //TODO hardcode robot size and kinect eye offset
        bboxSize = new double[3];
        eyeOffset = new double[3];
        eyeOffset[0] = eyeX;
        eyeOffset[1] = eyeY;
        eyeOffset[2] = eyeZ;

        bboxPos = new double[3];
        bboxRot = new double[3];

        centroid = new double[3];
        kinectEye = new double[3];

        svsCommands = new StringBuilder();
        create();
    }

    public Robot(pose_t robotData){
        name = null;

        //TODO hard code robot size and kinect eye offset
        bboxSize = new double[3];
        eyeOffset = new double[3];
        eyeOffset[0] = eyeX;
        eyeOffset[1] = eyeY;
        eyeOffset[2] = eyeZ;

        bboxPos = new double[3];
        bboxRot = new double[3];

        centroid = new double[3];
        kinectEye = new double[3];

        svsCommands = new StringBuilder();

        create(robotData);
    }

    // Name: Get/Set
    public String getName(){
        return name;
    }

    public void setName(String name){
    	this.name = name;
    	svsCommands.append(SVSCommands.changeProperty("robot", "name", name));
    }

    // Pose: Get
    // Pose is a 3-tuple consisting of XYZ
    public double[] getPos(){
        return bboxPos;
    }

    public void setPos(double[] pos){
    	this.bboxPos = pos;
    	svsCommands.append(SVSCommands.changePos("robot", pos));
    }

    // Rot: Get
    // Rot is a 3-tuple representing the rotation of the object in Roll-Pitch-Yaw
    public double[] getRot(){
    	return bboxRot;
    }

    // Size: Get
    // Size is a 3-tuple representing the size of the object's bounding box (XYZ)

    public double[] getSize(){
    	return bboxSize;
    }

    // Set Bounding Box Info
    public void setBBox(double[] rpy){
    	for(int i = 0; i < 3; i++){
    		this.bboxPos[i] = centroid[i]; //same in this representation
    		this.bboxRot[i] = rpy[i];
    	}

    	svsCommands.append(SVSCommands.changePos("robot", bboxPos));
    	svsCommands.append(SVSCommands.changeRot("robot", bboxRot));
    	svsCommands.append(SVSCommands.changePos("eye", kinectEye));
    }

    public synchronized void updateSVS(Agent agent){
    	if(svsCommands.length() > 0){
	    //System.out.println(svsCommands.toString());
        	agent.SendSVSInput(svsCommands.toString());
        	svsCommands = new StringBuilder();
    	}
    }

    public synchronized void create(){
    	svsCommands.append(SVSCommands.add("robot"));
		svsCommands.append(SVSCommands.add("eye"));
		svsCommands.append(SVSCommands.changeSize("robot", bboxSize));
    }

    public synchronized void create(pose_t robotData){
    	svsCommands.append(SVSCommands.add("robot"));
		svsCommands.append(SVSCommands.add("eye"));
		svsCommands.append(SVSCommands.changeSize("robot", bboxSize));

    	updateBbox(robotData);
		//updateProperties(robotData);
    }


    public synchronized void update(pose_t robotData){
    	updateBbox(robotData);
    //  updateProperties(robotData);
    }


    private void updateBbox(pose_t robotData){
   	    for(int i = 0; i < 3; i++){
   	     	centroid[i] = robotData.pos[i];
   	     	kinectEye[i] = robotData.pos[i] + eyeOffset[i];
   	    }
   	    // Convert from quat to RPY
   	    double[] quat = robotData.orientation;
   	    double[] rpy = LinAlg.quatToRollPitchYaw(quat);
    	setBBox(rpy);
    }
}
