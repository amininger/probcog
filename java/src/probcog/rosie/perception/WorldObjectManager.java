package probcog.rosie.perception;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.util.HashMap;
import java.util.Properties;

public class WorldObjectManager {
	
	private HashMap<Integer, WorldObject> objects;
	
	public WorldObjectManager(Properties props){
		objects = new HashMap<Integer, WorldObject>();
		
		String objectFile = props.getProperty("object-info-file");
		if(objectFile == null){
			System.err.println("No object-info-file specified in properties file");
		} else {
			readObjectInfoFile(objectFile);
		}
	}
	
	public WorldObject getObject(Integer tagID){
		return objects.get(tagID);
	}
	
	private void readObjectInfoFile(String filename){
        // This will reference one line at a time
        String line = null;

        try {
            // FileReader reads text files in the default encoding.
            FileReader fileReader = new FileReader(filename);

            // Always wrap FileReader in BufferedReader.
            BufferedReader bufferedReader = new BufferedReader(fileReader);

            while((line = bufferedReader.readLine()) != null) {
            	parseObjectInfo(line);
            }   

            // Always close files.
            bufferedReader.close();         
        }
        catch(FileNotFoundException ex) {
            System.err.println("WorldObjectManager: unable to open file " + filename);
        }
        catch(IOException ex) {
        	System.err.println("WorldObjectManager: error reading file " + filename);
        }
	}
	
	private void parseObjectInfo(String info){
		String[] params = info.split(" ");
		if(params.length < 2){
			return;
		}
		Integer tagID = new Integer(params[0]);
		if(objects.containsKey(tagID)){
			System.err.println("Trying to add multiple objects for tag id " + tagID.toString());
			return;
		}
		Integer numClasses = new Integer(params[1]);
		HashMap<String, String> classifications = new HashMap<String, String>();
		for(int i = 0; i < numClasses; i++){
			String name = params[2*i+2];
			String value = params[2*i+3];
			classifications.put(name, value);
		}
		objects.put(tagID, new WorldObject(tagID, classifications));
		System.out.println("Added object " + tagID.toString());
		System.out.println("  " + classifications.toString());
	}
}
