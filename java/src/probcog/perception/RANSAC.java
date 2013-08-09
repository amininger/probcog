package probcog.perception;

import java.util.*;

import april.jmat.*;

public class RANSAC
{

    /** Estimate the floor plane using RANSAC algorithm (assumes that the major
        plane in the image is the "floor").
        @param iterations is the number of iterations RANSAC is run for
        @return The characterizing coefficients for the floor plane
    **/

    public static double[] estimatePlane(ArrayList<double[]> points, int iterations, double percent)
    {
        if(points.size() == 0)
            return null;

        Random rand = new Random();
        int numPoints = points.size();
        double bestPlane[] = new double[4];  // Parameters of plane equation
        int bestFit = 0;                     // Most points that fit a guess plane
        int numSamples = (int)Math.floor(numPoints*percent);

        // Perform specified number of iterations
        for(int i=0; i<iterations; i++){
            int numFit = 0;
            // Choose three random points
            double[] r1 = points.get(rand.nextInt(numPoints));
            double[] r2 = points.get(rand.nextInt(numPoints));
            double[] r3 = points.get(rand.nextInt(numPoints));

            double[] p1 = new double[]{r1[0], r1[1], r1[2]};
            double[] p2 = new double[]{r2[0], r2[1], r2[2]};
            double[] p3 = new double[]{r3[0], r3[1], r3[2]};

            // Derive plane through all three points
            double[] p2p1 = LinAlg.subtract(p2, p1);
            double[] p3p1 = LinAlg.subtract(p3, p1);
            double[] pqr = LinAlg.normalize(LinAlg.crossProduct(p2p1, p3p1));
            if (pqr[2] < 0) {
                pqr[0] = -pqr[0];
                pqr[1] = -pqr[1];
                pqr[2] = -pqr[2];
            }
            double s = -(pqr[0]*r1[0] + pqr[1]*r1[1] + pqr[2]*r1[2]);
            double[] plane = new double[]{pqr[0], pqr[1], pqr[2], s};

            // Check whether a sample of points is within a threshold of the plane.
            for(int j = 0; j < numSamples; j++){
                double[] p = points.get(rand.nextInt(numPoints));
                if (pointToPlaneDist(new double[]{p[0], p[1], p[2]}, plane) < .005)
                    numFit ++;
            }

            // Compare new plane against last best, saving it if better
            if(numFit > bestFit && numFit > (numSamples/6)){
                bestFit = numFit;
                bestPlane = LinAlg.copy(plane);
            }
        }

        if(bestFit == 0)
            return new double[4];

        System.out.printf("Plane score: %d\n", bestFit);
        System.out.printf("Plane fit: %f _ %f _ %f _ %f \n", bestPlane[0], bestPlane[1], bestPlane[2], bestPlane[3]);
        return bestPlane;
    }

    /** Given a point and the coefficients for a plane, find the distance
     ** between them. **/
    public static double pointToPlaneDist(double[] point, double[] coef)
    {
        assert(point.length == 3 && coef.length == 4);

        double ax = coef[0]*point[0];
        double by = coef[1]*point[1];
        double cz = coef[2]*point[2];
        double a2 = coef[0]*coef[0];
        double b2 = coef[1]*coef[1];
        double c2 = coef[2]*coef[2];

        return Math.abs(ax+ by + cz + coef[3])/ Math.sqrt(a2 + b2 + c2);
    }

}
