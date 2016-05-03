package probcog.rosie;

import java.io.FileReader;
import java.io.IOException;
import java.util.Properties;

public class RosieConfig{
	public String agentName;

	public String agentSource;
	public String smemSource;

	public boolean spawnDebugger;
	public int watchLevel;
	public int throttleMS;
	
	public String speechFile;

	public Boolean writeLog;

	public RosieConfig(String configFile, boolean debug) throws IOException{

        // Load the properties file
        Properties props = new Properties();
		props.load(new FileReader(configFile));

        spawnDebugger = debug;

        agentName = props.getProperty("agent-name", "SoarAgent");
		agentSource = props.getProperty("agent-source", null);
		smemSource = props.getProperty("smem-source", null);

        try{
        	watchLevel = Integer.parseInt(props.getProperty("watch-level", "1"));
        } catch (NumberFormatException e){
        	watchLevel = 1;
        }

        try{
        	throttleMS = Integer.parseInt(props.getProperty("decision-throttle-ms", "0"));
        } catch(NumberFormatException e){
        	throttleMS = 0;
        }

		speechFile = props.getProperty("speech-file", "audio_files/sample");

        writeLog = props.getProperty("enable-log", "false").equals("true");
	}
}
