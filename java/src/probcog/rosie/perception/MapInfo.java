package probcog.rosie.perception;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Properties;

public class MapInfo {
	private HashSet<Region> regions;
	
	public MapInfo(Properties props){
		regions = new HashSet<Region>();
		
		String mapFile = props.getProperty("map-info-file");
		if(mapFile == null){
			System.err.println("MapInfo: No map-info-file specified in properties file");
		} else {
			readMapInfoFile(mapFile);
		}
	}
	
	public HashSet<Region> getRegions(double[] pos){
		HashSet<Region> regionSet = new HashSet<Region>();
		for(Region r : regions){
			if(r.pointInRegion(pos)){
				regionSet.add(r);
			}
		}
		return regionSet;
	}
	
	private void readMapInfoFile(String filename){
        // This will reference one line at a time
        String line = null;

        try {
            // FileReader reads text files in the default encoding.
            FileReader fileReader = new FileReader(filename);

            // Always wrap FileReader in BufferedReader.
            BufferedReader bufferedReader = new BufferedReader(fileReader);

            while((line = bufferedReader.readLine()) != null) {
            	try{
            		Region r = new Region(line);
            		regions.add(r);
            	} catch(ParseException e){
            		
            	}
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
}
	