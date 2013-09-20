package probcog.perception;

import java.awt.Rectangle;
import java.io.IOException;
import java.util.ArrayList;
import java.util.HashMap;
import java.util.Map;

import probcog.sensor.Sensor;
import probcog.sensor.SimKinectSensor;
import probcog.sensor.SimKinectSensor.SimPixel;
import probcog.sim.SimLocation;
import probcog.sim.SimObjectPC;
import april.config.Config;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.TimeUtil;

public class SimKinectSegment implements Segmenter
{
    private ArrayList<Sensor> sensors = new ArrayList<Sensor>();
    private SimKinectSensor kinect;
    
    private final static int PIXEL_STRIDE = 2; 
    // Number of pixels to move for each sample
    // Higher values produce a less dense image
    
    // Set up some "Random" colors to draw the segments
    static int[] colors = new int[]{0xff3300CC, 0xff9900CC, 0xffCC0099, 0xffCC0033,
        0xff0033CC, 0xff470AFF, 0xff7547FF, 0xffCC3300,
        0xff0099CC, 0xffD1FF47, 0xffC2FF0A, 0xffCC9900,
        0xff00CC99, 0xff00CC33, 0xff33CC00, 0xff99CC00};

    public SimKinectSegment(Config config_, SimWorld world) throws IOException
    {
    	
        kinect = new SimKinectSensor(world);

        sensors.add(kinect);    // XXX
    }
    
    public void scanWholeScene(HashMap<SimObject, PointCloud> pointClouds){
    	PointCloud lastPC = null;
    	SimObject lastObj = null;    	
    	
        int height = kinect.getHeight();
        int width = kinect.getWidth();

        // Go through the camera and get points for each pixel 
        // Keep the points separated based on the object that they hit
        for (int y = 0; y < height; y += PIXEL_STRIDE) {
            for (int x = 0; x < width; x += PIXEL_STRIDE) {
            	scanPoint(x, y, pointClouds);
            	/*SimPixel pixel = kinect.getPixel(x, y);
            	if(pixel.target == null){
            		// Ray didn't hit anything
            		continue;
            	}
            	if(pixel.target instanceof SimLocation){
            		// We aren't segmenting sim locations
            		continue;
            	}
            	if(pixel.target == lastObj){
            		// Small optimization to avoid some map searching for consecutive rays hitting the same object
            		lastPC.addPoint(pixel.point);
            		continue;
            	} 
            	lastObj = pixel.target;
            	lastPC = pointClouds.get(lastObj);
            	if(lastPC == null){
            		lastPC = new PointCloud();
            		pointClouds.put(lastObj, lastPC);
            	}
        		lastPC.addPoint(pixel.point);
        		*/
            	
            }
        }
    }
    
    /********
     * scanSceneSubset(Rectangle bounds, int depth, HashMap<SimObject, PointCloud> pointClouds)
     * 
     * Scans a subset of the scene as defined by the bounds (in pixel space)
     * Uses depth as a resolution, lower numbers (minimum of 1) result in a finer scan
     * Divides the region into quadrants and scans each separately.
     * If a quadrant scan results in a positive result (hit an object), then this 
     *    recursively calls itself on the quadrant with a finer resolution (lower depth)
     *   
     * This is an optimization so that instead of every pixel being scanned, only regions containing
     * objects are scanned at a fine resolution
     * 
     * Note: starting this at a higher depth may result in more efficiency, but increases the chance that an object
     *   might be missed (e.g. for depth = 5, the stride size is 2^5 = 32, so only every 32 pixels are considered, 
     *       thus an object that is smaller than 32x32 may be missed 
     */
    public void scanSceneSubset(Rectangle bounds, int depth, HashMap<SimObject, PointCloud> pointClouds){
    	int midX = bounds.x + (int)Math.floor(bounds.width/2.0f);
    	int midY = bounds.y + (int)Math.floor(bounds.height/2.0f);
    	int leftSize = midX - bounds.x;
    	int rightSize = bounds.x + bounds.width - midX;
    	int topSize = midY - bounds.y;
    	int botSize = bounds.y + bounds.height - midY;
    	
    	Rectangle topLeft = new Rectangle(bounds.x, bounds.y, leftSize, topSize);
    	Rectangle topRight = new Rectangle(midX, bounds.y, rightSize, topSize);
    	Rectangle botLeft = new Rectangle(bounds.x, midY, leftSize, botSize);
    	Rectangle botRight = new Rectangle(midX, midY, rightSize, botSize);
    	
    	if(scanRectangle(topLeft, depth, pointClouds) && depth > 1){
    		scanSceneSubset(topLeft, depth-1, pointClouds);
    	}
    	if(scanRectangle(topRight, depth, pointClouds) && depth > 1){
    		scanSceneSubset(topRight, depth-1, pointClouds);
    	}
    	if(scanRectangle(botLeft, depth, pointClouds) && depth > 1){
    		scanSceneSubset(botLeft, depth-1, pointClouds);
    	}
    	if(scanRectangle(botRight, depth, pointClouds) && depth > 1){
    		scanSceneSubset(botRight, depth-1, pointClouds);
    	}
    }
    
    /*********
     * boolean scanRectangle(Rectangle bounds, int depth, HashMap<SimObject, PointCloud> pointClouds)
     * 
     * Scans the scene over the given rectangle (in pixel space)
     * Uses the depth to determine how finely to scan, it uses a stride of 2^depth 
     *   (i.e. depth 2 means you scan every 4th pixel)
     * 
     * If any of the scans within the rectangle region hit an object, this returns true
     * If no object was hit, returns false
     */
    public boolean scanRectangle(Rectangle bounds, int depth, HashMap<SimObject, PointCloud> pointClouds){
    	boolean hitObject = false;
    	int startX = bounds.x + (int)Math.pow(2, depth) - 1;
    	int startY = bounds.y + (int)Math.pow(2, depth) - 1;
    	int endX = bounds.x + bounds.width;
    	int endY = bounds.y + bounds.height;
    	int stride = (int)Math.pow(2, depth);
    	
    	for(int x = startX; x < endX; x += stride){
    		for(int y = startY; y < endY; y += stride){
    			hitObject = hitObject | scanPoint(x, y, pointClouds);
    		}
    	}
    	
    	return hitObject;    	
    }
    
    /**********
     * boolean scanPoint(int px, int py, HashMap<SimObject, PointCloud> pointClouds)
     * 
     * Scans the scene at the given point (in pixel space) using ray tracing from the simulated kinect
     * If it hits an object, it adds it to the appropriate point cloud in the supplied hashmap and return true
     * If it does not hit anything, it returns false
     */
    int numScans = 0;
    public boolean scanPoint(int px, int py, HashMap<SimObject, PointCloud> pointClouds){
		numScans++;
    	SimPixel pixel = kinect.getPixel(px, py);
    	if(pixel.target == null){
    		// Ray didn't hit anything
    		return false;
    	}
    	if(pixel.target instanceof SimLocation){
    		// We aren't segmenting sim locations
    		return false;
    	}
    	PointCloud pc = pointClouds.get(pixel.target);
    	if(pc == null){
    		pc = new PointCloud();
    		pointClouds.put(pixel.target, pc);
    	}
		pc.addPoint(pixel.point);
		return true;
    }
    
    
    public ArrayList<Obj> getSegmentedObjects(){
    	HashMap<SimObject, PointCloud> pointClouds = new HashMap<SimObject, PointCloud>();  
    	numScans = 0;
    	
    	
    	// AM: Optimization here, tries to do a coarse initial scan over the scene, only doing
    	//      finer scans when an object is hit. see scanSceneSubset for more details
    	// Empirical testing on the simulator shows a 8-10 times speedup
    	boolean scanSubsets = true;
    	if(scanSubsets){
    		int height = (int)kinect.getHeight() / 4;
            int width = (int)kinect.getWidth() / 4;
            // We start scanning by dividing the view region into 4x4 grid and scan at depth 3 = stride of 8
            for(int x = 0; x < 4; x++){
            	for(int y = 0; y < 4; y++){
            		scanSceneSubset(new Rectangle(x * width, y * height, width, height), 3, pointClouds);
            	}
            }
    	} else {
    		scanWholeScene(pointClouds);
    	}
        
        // Turn the segmented point clouds into Obj's
    	ArrayList<Obj> segmentedObjs = new ArrayList<Obj>();
    	for(Map.Entry<SimObject, PointCloud> entry : pointClouds.entrySet()){
    		if(entry.getValue().getPoints().size() == 0){
    			continue;
    		} else if(entry.getKey() instanceof SimObjectPC && 
    				!((SimObjectPC)entry.getKey()).getVisible()){
    			continue;
    		}
    		Obj obj = new Obj(false, entry.getValue());
    		obj.setSourceSimObject(entry.getKey());
    		segmentedObjs.add(obj);
    	}
    	
    	return segmentedObjs;    	
    }

    // === Provide access to the raw sensor === //
    // XXX This seems a bit hacky to provide...
    public ArrayList<Sensor> getSensors()
    {
        return sensors;
    }
}
