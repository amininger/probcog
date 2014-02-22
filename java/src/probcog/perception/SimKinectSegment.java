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

    
    public ArrayList<Obj> getSegmentedObjects(){
    	SimPixel[] pixels = kinect.getAllPixels();

    	HashMap<SimObject, PointCloud> pointClouds = new HashMap<SimObject, PointCloud>();  
    	for(SimPixel pixel : pixels){
    		if(pixel == null || pixel.target == null){
    			continue;
    		}
    		PointCloud pc = pointClouds.get(pixel.target);
    		if(pc == null){
    			pc = new PointCloud();
    			pointClouds.put(pixel.target, pc);
    		}
    		pc.addPoint(pixel.point);
    	}
        
        // Turn the segmented point clouds into Obj's
    	ArrayList<Obj> segmentedObjs = new ArrayList<Obj>();
    	for(Map.Entry<SimObject, PointCloud> entry : pointClouds.entrySet()){
    		if(entry.getValue().getPoints().size() == 0){
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
