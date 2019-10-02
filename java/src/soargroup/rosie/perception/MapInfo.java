package soargroup.rosie.perception;

import java.io.BufferedReader;
import java.io.FileNotFoundException;
import java.io.FileReader;
import java.io.IOException;
import java.text.ParseException;
import java.util.HashMap;
import java.util.HashSet;
import java.util.Properties;

import april.util.StringUtil;

public class MapInfo {
	private HashSet<Region> regions;
	private double[] robotPos;
	
	public MapInfo(Properties props){
		regions = new HashSet<Region>();
		robotPos = new double[]{ 0.0, 0.0 };
		
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

    public HashSet<Region> getAllRegions(){
        return regions;
    }
    
    public double[] getRobotPos(){
    	return robotPos;
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
            		String[] words = line.split(" ");
					if(words.length >= 3 && words[0].equals("robot")){
						robotPos[0] = new Double(words[1]);
						robotPos[1] = new Double(words[2]);
					} else {
						Region r = new Region(line);
						regions.add(r);
					}
            	} catch(ParseException e){
            		
            	}
            }   

            // Always close files.
            bufferedReader.close();         
        }
        catch(FileNotFoundException ex) {
            System.err.println("MapInfo: unable to open file " + filename);
        }
        catch(IOException ex) {
        	System.err.println("MapInfo: error reading file " + filename);
        }
	}
}
	
