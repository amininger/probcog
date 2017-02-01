package probcog.rosie.perception;

import java.util.ArrayList;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Map;
import java.util.regex.Matcher;
import java.util.regex.Pattern;

import edu.umich.rosie.soar.ISoarObject;
import edu.umich.rosie.soar.SVSCommands;
import sml.Agent;
import sml.Identifier;
import probcog.util.BoundingBox;
import april.jmat.LinAlg;
import april.util.TimeUtil;

/**
 * A single Object in the world, can be created using either a sensable or an object_data_t
 *
 * @author mininger
 *
 */
public class WorldObject implements ISoarObject
{
    // Handle of the object
    protected String handle;

    // Information about the bounding box
    // Center of the bounding box (XYZ)
    protected double[] bboxPos;

    // Orientation of the bounding box (RPY)
    protected double[] bboxRot;

    // Size of the bounding box (dX, dY, dZ)
    protected double[] bboxSize;

    protected double[] centroid;

    protected ArrayList<ObjectData> lastData;

    protected Map<String, ObjectProperty> perceptualProperties;
    protected Map<String, ObjectProperty> stateProperties;

    protected HashSet<ObjectProperty> propsToRemove;

    private WorldModel world;

    private boolean gotPropUpdate = false;
    private boolean gotBboxUpdate = false;
    private boolean gotPoseUpdate = false;

    public WorldObject(WorldModel world, String handle, ArrayList<ObjectData> objDatas){
    	this.world = world;
        this.handle = handle;
        bboxPos = new double[3];
        bboxRot = new double[3];
        bboxSize = new double[3];
        centroid = new double[3];
        perceptualProperties = new HashMap<String, ObjectProperty>();
        stateProperties = new HashMap<String, ObjectProperty>();
        propsToRemove = new HashSet<ObjectProperty>();

        lastData = objDatas;
        updateBbox(objDatas);
        updateProperties(objDatas);
    }

    // Handle: Get
    public int getHandle(){
        return (new Integer(handle));
    }

    public String getHandleStr(){
    	return handle;
    }

    public Integer getPerceptionId(){
        Integer pid = new Integer(handle);
    	for(ObjectData objDat : lastData){
    		if(objDat.getID() == pid){
    			return pid;
    		}
    	}

    	if(lastData.size() > 0){
    		return lastData.get(0).getID();
    	}
    	// This should never happen
    	return null;
    }

    // Pose: Get
    // Pose is a 3-tuple consisting of XYZ

    public double[] getPos(){
        return bboxPos;
    }

    public void setPos(double[] pos){
    	this.bboxPos = pos;
    	gotPoseUpdate = true;
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

    public ArrayList<ObjectData> getLastDatas(){
    	return lastData;
    }

    // Set Bounding Box Info
    public void setBBox(double[] xyzrpy, double[] size){
    	for(int i = 0; i < 3; i++){
    		this.bboxPos[i] = xyzrpy[i];
    		this.bboxRot[i] = xyzrpy[3+i];
    		this.bboxSize[i] = size[i];
    	}

    	gotBboxUpdate = true;
    }
//
//    public void moveObject(double[] newPos){
//    	double[] diff = LinAlg.subtract(newPos, bboxPos);
//    	this.bboxPos = newPos;
//    	this.centroid = LinAlg.add(centroid, diff);
//    	gotPoseUpdate = true;
//    }

    public void update(ArrayList<ObjectData> objDatas){
    	lastData = objDatas;
        updateBbox(objDatas);
        updateProperties(objDatas);
    }

    private void updateProperties(ArrayList<ObjectData> objDatas){
    	if(objDatas.size() > 1){
    		// We don't actually know how to combine the properties, so we report as unknown for now
    		HashMap<String, Double> unknownValues = new HashMap<String, Double>();
    		unknownValues.put("unknown", 0.0);

    		for(ObjectProperty p : perceptualProperties.values()){
    			p.updateProperty(unknownValues);
    		}

    		for(ObjectProperty p : stateProperties.values()){
    			p.updateProperty("unknown");
    		}
    	} else {
    		// Just one object, update using that
    		ObjectData objectData = objDatas.get(0);
    		ArrayList<String> propsToDelete = new ArrayList<String>(perceptualProperties.keySet());

            for(CategorizedData category : objectData.getCatDat()) {
                String propName = PerceptualProperty.getPropertyName(category.getCatType());
                if(perceptualProperties.containsKey(propName)){
                    propsToDelete.remove(propName);
                    perceptualProperties.get(propName).updateProperty(category);
                } else {
                    PerceptualProperty pp = new PerceptualProperty(category);
                    perceptualProperties.put(propName, pp);
                }
            }

            for(String propName : propsToDelete){
                PerceptualProperty pp = perceptualProperties.get(propName);
                propsToRemove.add(pp);
                perceptualProperties.remove(propName);
            }

    		propsToDelete = new ArrayList<String>(stateProperties.keySet());

    		for(String stateProp : objectData.getStateValues()){
        		String[] nameValSplit = stateProp.split("=");
        		if(nameValSplit.length < 2){
        			continue;
        		}
        		String propName = nameValSplit[0].toLowerCase();
        		String propVal = nameValSplit[1].toLowerCase();
        		if(stateProperties.containsKey(propName)){
        			propsToDelete.remove(propName);
        			stateProperties.get(propName).updateProperty(propVal);
        		} else {
        			ObjectProperty sp = new ObjectProperty(propName, ObjectProperty.STATE_TYPE, propVal);
        			stateProperties.put(propName, sp);
        		}
    		}
    		for (String propName : propsToDelete){
    			ObjectProperty sp = stateProperties.get(propName);
    			propsToRemove.add(sp);
    			stateProperties.remove(propName);
    		}
    	}
    	gotPropUpdate = true;
    }

    private void updateBbox(ArrayList<ObjectData> objDatas){
    	if(objDatas.size() == 1){
    		// Nothing fancy, just update using the given information
    		ObjectData objectData = objDatas.get(0);
   	     	setBBox(objectData.getBBoxPos(), objectData.getBBoxDim());

   	     	for(int i = 0; i < 3; i++){
   	     		centroid[i] = objectData.getBBoxPos(i);
   	     	}
    	} else {
    		// Combine multiple bounding boxes into 1,
    		// we generate all the points on the corners of each bbox
    		// then calculate a new oriented bbox based on those
    		ArrayList<double[]> points = new ArrayList<double[]>();

    		for(ObjectData objectData : objDatas){
    			// Position + Rotation Matrix
    			double[][] pr = LinAlg.xyzrpyToMatrix(objectData.getBBoxPos());
    			// Position + Rotation + Scaling Matrix
    			double[][] prs = LinAlg.matrixAB(pr,
                                                 LinAlg.scale(objectData.getBBoxDim(0)/2,
                                                              objectData.getBBoxDim(1)/2,
                                                              objectData.getBBoxDim(2)/2));

    			// Center of the box
    			double[] p = new double[]{objectData.getBBoxPos(0),
                                          objectData.getBBoxPos(1),
                                          objectData.getBBoxPos(2)};
    			// Each Axis of the box
    			double[] x = new double[]{prs[0][0], prs[1][0], prs[2][0]};
    			double[] y = new double[]{prs[0][1], prs[1][1], prs[2][1]};
    			double[] z = new double[]{prs[0][2], prs[1][2], prs[2][2]};

    			for(int i = -1; i <= 1; i += 2){
    				for(int j = -1; j <= 1; j += 2){
    					for(int k = -1; k <= 1; k += 2){
    						// Each corner of the box
    						double[] v = LinAlg.add(LinAlg.scale(x, i),
                                                    LinAlg.add(LinAlg.scale(y, j),
                                                               LinAlg.scale(z, k)));
    						points.add(LinAlg.add(p, v));
    					}
    				}
    			}
    		}

			BoundingBox bbox = BoundingBox.getMinimalXY(points);

    		setBBox(bbox.xyzrpy, bbox.lenxyz);
    		for(int i = 0; i < 3; i++){
    			centroid[i] = bbox.xyzrpy[i];
    		}
    	}
    }

    /***********************************************************
     * Methods for managing working memory structures
     ***********************************************************/

    protected Identifier objId;
    private boolean added = false;

    public boolean isAdded(){
    	return added;
    }

    public void addToWM(Identifier parentId){
    	if(added){
    		removeFromWM();
    	}
		objId = parentId.CreateIdWME("object");
    	objId.CreateStringWME("object-handle", new String(handle));

    	for(ObjectProperty pp : perceptualProperties.values()){
    		pp.addToWM(objId);
    	}
    	for(ObjectProperty sp : stateProperties.values()){
    		sp.addToWM(objId);
    	}

    	StringBuilder svsCommands = new StringBuilder();
    	svsCommands.append(SVSCommands.addBox(getHandleStr(), bboxPos, bboxRot, bboxSize));
		svsCommands.append(SVSCommands.addTag(getHandleStr(), "object-source", "perception"));
		world.getAgent().SendSVSInput(svsCommands.toString());
    	added = true;
    }

    public void updateWM(){
    	if(!added){
    		return;
    	}

    	if(gotPropUpdate){
	    	// Update Perceptual Properties
	    	for(ObjectProperty pp : perceptualProperties.values()){
	    		if(pp.isAdded()){
	    			pp.updateWM();
	    		} else {
	    			pp.addToWM(objId);
	    		}
	    	}
	    	for(ObjectProperty sp : stateProperties.values()){
	    		if(sp.isAdded()){
	    			sp.updateWM();
	    		} else {
	    			sp.addToWM(objId);
	    		}
	    	}
	    	for(ObjectProperty pp : propsToRemove){
	    		pp.removeFromWM();
	    	}
	    	propsToRemove.clear();
	    	gotPropUpdate = false;
    	}

    	// Update SVS
		StringBuilder svsCommands = new StringBuilder();
    	if(gotPoseUpdate || gotBboxUpdate){
    		svsCommands.append(SVSCommands.changePos(getHandleStr(), bboxPos));
    		gotPoseUpdate = false;
    	}
    	if(gotBboxUpdate){
    		svsCommands.append(SVSCommands.changeRot(getHandleStr(), bboxRot));
    		svsCommands.append(SVSCommands.changeScale(getHandleStr(), bboxSize));
    		gotBboxUpdate = false;
    	}
    	if(svsCommands.length() > 0){
        	world.getAgent().SendSVSInput(svsCommands.toString());
        	svsCommands = new StringBuilder();
    	}
    	world.getAgent().SendSVSInput(svsCommands.toString());
    }

    public void removeFromWM(){
    	if(!added){
    		return;
    	}

    	StringBuilder svsCommands = new StringBuilder();
    	svsCommands.append(SVSCommands.delete(getHandleStr()));
    	world.getAgent().SendSVSInput(svsCommands.toString());
    	for(ObjectProperty pp : perceptualProperties.values()){
    		pp.removeFromWM();
    	}
    	for(ObjectProperty sp : stateProperties.values()){
    		sp.removeFromWM();
    	}
    	objId.DestroyWME();
    	objId = null;
    	added = false;
    }
}
