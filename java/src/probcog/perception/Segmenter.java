package probcog.perception;

import java.util.ArrayList;

import probcog.sensor.Sensor;

public interface Segmenter {
	
	// Segments the scene into individual objects
	public ArrayList<Obj> getSegmentedObjects();
	
	// Gets the sensors used to provide data to the segmenter
	public ArrayList<Sensor> getSensors();

}
