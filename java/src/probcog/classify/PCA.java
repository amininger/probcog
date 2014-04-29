package probcog.classify;

import java.awt.*;
import java.io.*;
import java.util.*;

import april.jmat.LinAlg;
import april.util.*;

import probcog.util.*;

public class PCA
{
    /** A new version of get feature. This one takes a start point and direction
     *  vector and a set of all of our points (assumed to be normalized into a
     *  unit box already.) We quantize them to an arbitrary resolution and
     *  then sweep in direction until we hit a filled box. We return the
     *  sweep distance as the feature
     *
     */
    public static double getFeature(ArrayList<double[]> pixels, double[] start, double[] dir)
    {
        double res = 5.0;
        GridMap gm = GridMap.makeMeters(0, 0, 1, 1, 1.0/res, 0);
        for (double[] p: pixels) {
            gm.setValue(p[0], p[1], (byte)1);
        }

		final double increment = .5*gm.metersPerPixel;
		double x = start[0];
		double y = start[1];
        //System.out.println(x + " " +y);

        while (true) {
            int val = gm.getValue(x,y);
            if (val != 0)
                return LinAlg.distance(start, new double[] {x,y});

            x += dir[0]*increment;
            y += dir[1]*increment;
            double dist = LinAlg.distance(start, new double[] {x,y});
            //System.out.println("\t" + x + " " + y + " " + dist);
            if (dist > 1.0)
                return 1.0;
		}
    }
    

    /* Assumes that pixels is an array of double arrays with x and y values
     *   These values should be between 0 and 1 inclusive
     * 
     * This divides the unit region into rows and columns based on featuresPerSide
     *   It then calculates the min and max pixels within that row/column
     *   And uses that as a feature
     */
    public static ArrayList<Double> getFeatures(ArrayList<double[]> pixels, int featuresPerSide){
    	ArrayList<Double> features = new ArrayList<Double>();
    	double[] rowMins = new double[featuresPerSide];
    	double[] rowMaxs = new double[featuresPerSide];
    	double[] colMins = new double[featuresPerSide];
    	double[] colMaxs = new double[featuresPerSide];
    	
//    	boolean[][] cells = new boolean[featuresPerSide][featuresPerSide];
//    	for(int r = 0; r < featuresPerSide; r++){
//    		for(int c = 0; c < featuresPerSide; c++){
//    			cells[r][c] = false;
//    		}
//    	}
    	
    	for(int i = 0; i < featuresPerSide; i++){
    		rowMins[i] = colMins[i] = 1;
    		rowMaxs[i] = colMaxs[i] = 0;
    	}
    	for(double[] pixel : pixels){
    		int col = (int)(featuresPerSide * pixel[0] * .9999); // Slightly scaled down so col != featuresPerSide if pixel[0] == 1
    		int row = (int)(featuresPerSide * pixel[1] * .9999);
    		rowMaxs[row] = Math.max(pixel[0], rowMaxs[row]);
    		rowMins[row] = Math.min(pixel[0], rowMins[row]);
    		colMaxs[col] = Math.max(pixel[1], colMaxs[col]);
    		colMins[col] = Math.min(pixel[1], colMins[col]);
//    		cells[row][col] = true;
    	}
    	for(int i = 0; i < featuresPerSide; i++){
    		features.add(colMins[i]);
    	}
    	for(int i = 0; i < featuresPerSide; i++){
    		features.add(rowMaxs[i]);
    	}
    	for(int i = 0; i < featuresPerSide; i++){
    		features.add(colMaxs[i]);
    	}
    	for(int i = 0; i < featuresPerSide; i++){
	   		features.add(rowMins[i]);
    	}
//
//    	for(int r = 0; r < featuresPerSide; r++){
//    		for(int c = 0; c < featuresPerSide; c++){
//    			System.out.print(cells[r][c] ? 'X' : ' ');
//    		}
//    		System.out.print("\n");
//    	}
//    	
//    	System.out.print("\n");
//    	
//      
//      for(Double d : features){
//      	System.out.print(String.format("%.3f ", d));
//      }
//      System.out.println("");

    	
    	return features;
    }


//    /** Modified get faetures method tossing out buffered image. Also, doesn't
//     * do real PCA...*/
//    public static ArrayList<Double> getFeatures(ArrayList<double[]> pixels, int numFeatures)
//    {
//	    ArrayList<Double> features = new ArrayList<Double>();
//	    //features.add((proj1[1] - proj1[0])/(proj2[1] - proj2[0]));
//        // Original version set 2 values. Up and down. Now also do left/right
//	    for(int i = 0; i < 4; i++){
//	    	// Start at bottom-left or top-left
//	    	double[] start = new double[2];
//	    	//start[0] = leftCenter[0] + proj2[i]*v2[0];
//	    	//start[1] = leftCenter[1] + proj2[i]*v2[1];
//            if (i < 2) {
//                start[1] = i;
//            } else {
//                start[0] = i-2;
//            }
//
//	    	// Direction is either up or down
//	    	//double[] dir = LinAlg.scale(v2, (i == 0 ? 1 : -1));
//            double[] dir;
//            if (i < 2) {
//                dir = i == 0 ? new double[] {0, 1} : new double[] {0, -1};
//            } else {
//                dir = (i-2) == 0 ? new double[] {1, 0} : new double[] {-1, 0};
//            }
//
//	    	// Pick points evenly along the line
//	    	for (float perc = .02f; perc <= .98f; perc += .95f/(numFeatures-1)) {
//	    		double[] pt = new double[2];
//	    		//pt[0] = start[0] + perc*(proj1[1] - proj1[0])*v1[0];
//	    		//pt[1] = start[1] + perc*(proj1[1] - proj1[0])*v1[1];
//                pt[0] = start[0];
//                pt[1] = start[1];
//                if (i < 2) {
//                    pt[0] += perc;
//                } else {
//                    pt[1] += perc;
//                }
//	    		//features.add(getFeature(img, pt, dir)/(proj2[1] - proj2[0]));
//                features.add(getFeature(pixels, pt, dir));
//	    	}
//	    }
//
//    	return features;
//    }
}
