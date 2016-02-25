package probcog.rosie.perception;

import java.text.ParseException;

public class Region{
	public String id;
	public double x;
	public double y;
	public double rot;
	public double width;
	public double length;
	public String label;
	
	public Region(String line) throws ParseException{
		String[] params = line.split(" ");
		int offset = 0;
		// Param 1: id
		if(params.length < 1){
			throw new ParseException("MapInfo: need region id", offset);
		}
		id = params[0];
		offset += params[0].length() + 1;

		// Param 2: x coordinate of center
		if(params.length < 2){
			throw new ParseException("MapInfo: need x coordinate", offset);
		}
		x = Double.parseDouble(params[1]);
		offset += params[1].length() + 1;

		// Param 3: y coordinate of center
		if(params.length < 3){
			throw new ParseException("MapInfo: need y coordinate", offset);
		}
		y = Double.parseDouble(params[2]);
		offset += params[2].length() + 1;

		// Param 4: rotation (in radians) of region
		if(params.length < 4){
			throw new ParseException("MapInfo: need rotation (in radians)", offset);
		}
		rot = Double.parseDouble(params[3]);
		offset += params[3].length() + 1;

		// Param 5: width of region (x direction)
		if(params.length < 5){
			throw new ParseException("MapInfo: need width of region (x direction)", offset);
		}
		width = Double.parseDouble(params[4]);
		offset += params[4].length() + 1;

		// Param 6: length of region (y direction)
		if(params.length < 6){
			throw new ParseException("MapInfo: need length of region (y direction)", offset);
		}
		length = Double.parseDouble(params[5]);
		offset += params[5].length() + 1;
		
		// Param 7: label for the region (e.g. room, hall)
		if(params.length < 7){
			throw new ParseException("MapInfo: need label for the region", offset);
		}
		label = params[6];
		offset += params[6].length() + 1;
	}
	
	public double getDistanceSq(double[] p){
		double dx = p[0] - x;
		double dy = p[1] - y;
		return (dx*dx + dy*dy);
	}
	
	public boolean pointInRegion(double[] p){
		double dx = p[0] - x;
		double dy = p[1] - y;
		double dist = Math.sqrt(dx*dx + dy*dy);
		double theta = Math.atan2(dy, dx);
		double localTheta = theta - rot;
		double xproj = dist * Math.cos(localTheta);
		double yproj = dist * Math.sin(localTheta);
		return (Math.abs(xproj) < width/2 && Math.abs(yproj) < length/2);
	}
}
