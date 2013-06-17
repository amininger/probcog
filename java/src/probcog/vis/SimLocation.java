package probcog.vis; // XXX - Should it go here?

import java.awt.Color;
import java.io.IOException;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.util.Util;

public class SimLocation implements SimObject
{
    private VisObject model;
    private Shape shape;

    private double[] pose;
    private double size;

    private String name;
    private int id;
    private Color color;

    private HashMap<String, String[]> possibleStates;
    private HashMap<String, String> currentStates;


    public SimLocation(SimWorld sw)
    {
        size = .1;
        shape = new BoxShape(new double[]{2*size, 2*size, 0});
        id = Util.nextID();
        possibleStates = new HashMap<String, String[]>();
        currentStates = new HashMap<String, String>();
    }

    public void setColor(int[] rgb)
    {
        assert(rgb.length == 3);
        color = new Color(rgb[0], rgb[1], rgb[2]);
    }

    public Color getColor()
    {
        return color;
    }

    public int getID()
    {
        return id;
    }

    public VisObject getVisObject()
    {
        return constructModel();
    }

    public Shape getShape()
    {
        return shape;
    }

    public void setPose(double[] pose)
    {
        this.pose = pose;
    }

    public void setPose(double[][] poseMatrix)
    {
		pose = LinAlg.matrixToXyzrpy(poseMatrix);
	}

    public double[][] getPose()
    {
		return LinAlg.xyzrpyToMatrix(pose);
    }

    public void setName(String name)
    {
        this.name = name;
    }

    public String getName()
    {
        return name;
    }

    public void setPossibleStates(HashMap<String, String[]> possible)
    {
        possibleStates = possible;
    }

    public void setCurrentStates(HashMap<String, String> current)
    {
        currentStates = current;
    }


    public String getProperties()
    {
		String props = String.format("ID=%d,NAME=%s,", id, name);

        if(currentStates.size() > 0){
            StringBuilder properties = new StringBuilder();
            for(Map.Entry<String, String> state : currentStates.entrySet()){
                properties.append(String.format("%s=%s,", state.getKey(), state.getValue()));
            }
            props += properties.substring(0, properties.length() - 1) + ",";
        }

		props += String.format("POSE=[%f %f %f %f %f %f],", pose[0], pose[1], pose[2], pose[3], pose[4], pose[5]);
		props += String.format("BBOX=[%f %f %f %f %f %f]", -size, -size, -size, size, size, size);

		return props;
	}


    public void setState(String keyValueString)
    {
        String[] keyValuePair = keyValueString.split("=");
        if(keyValuePair.length < 2)
            return;

        String stateName = keyValuePair[0].toLowerCase();
        String newState = keyValuePair[1].toLowerCase();

        String[] states = possibleStates.get(stateName);
        if(states == null) {
            return;
        }
        else if(currentStates.containsKey(newState)) {
            currentStates.put(stateName, newState);
        }
    }

    private VisObject constructModel()
    {
        ArrayList<Object> objs = new ArrayList<Object>();

        // The larger box making up the background of the object
		objs.add(new VisChain(LinAlg.translate(0,0,.001),
				LinAlg.scale(size), new VzRectangle(new VzMesh.Style(color))));

		// The name of the location
		objs.add(new VisChain(LinAlg.rotateZ(Math.PI/2), LinAlg.translate(0,-.8*size,.007),
                LinAlg.scale(0.002),
                new VzText(VzText.ANCHOR.CENTER, String.format("<<black>> %s", name))));

        // The smaller inner box is only drawn if there is a door and it's open
        if(currentStates.containsKey("door") && currentStates.get("door").equals("open")) {
			objs.add(new VisChain(LinAlg.translate(0,0,.004),
					LinAlg.scale(size*.9), new VzRectangle(new VzMesh.Style(Color.DARK_GRAY))));
		}

		return new VisChain(objs.toArray());
    }

	public void read(StructureReader ins) throws IOException
    {
		double[] xy = ins.readDoubles();
    	pose = new double[]{xy[0], xy[1], 0, 0, 0, 0};
        name = ins.readString();

        // Set color
    	int[] colors = ins.readInts();
    	color = new Color(colors[0], colors[1], colors[2]);

        // Set the allowed properties and the current properties
        int numProperties = ins.readInt();
        String[] properties = new String[numProperties];
        for(int i = 0; i < numProperties; i++){
        	properties[i] = ins.readString();
        }
        for(String prop : properties) {
            String[] nameVals = prop.split("=");
            if(nameVals.length >= 2) {
                String[] allowedStates = nameVals[1].split(",");
                for(int i=0; i<allowedStates.length; i++)
                    allowedStates[i] = allowedStates[i].toLowerCase();

                possibleStates.put(nameVals[0].toLowerCase(), allowedStates);
                currentStates.put(nameVals[0].toLowerCase(), allowedStates[0]);
            }
        }

        shape = new BoxShape(new double[]{2*size, 2*size, 0});
    }


    public void write(StructureWriter outs) throws IOException
    {
    	outs.writeComment("XY");
        outs.writeDoubles(new double[]{pose[0], pose[1]});
        outs.writeString(name);

        outs.writeInts(new int[]{color.getRed(), color.getGreen(), color.getBlue()});

        outs.writeInt(possibleStates.size());
        for(String property : possibleStates.keySet()) {
            String nameValues = property + "=" + currentStates.get(property);
            for(String state : possibleStates.get(property)) {
                if(!state.equals(currentStates.get(property))) {
                    nameValues += "," + state;
                }
            }
            outs.writeString(nameValues);
        }
    }

    // Override for SimObject
	public void setRunning(boolean arg0)
    {
	}
}