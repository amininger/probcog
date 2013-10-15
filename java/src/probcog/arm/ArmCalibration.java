package probcog.arm;

import java.io.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.util.*;

/** Given a set correspondences between requested arm positions
 *  and actual executions of those requests, provides a function that
 *  returns the actual position you should request the arm go to best
 *  achieve your desired results.
 */
public class ArmCalibration
{
    // XXX Tune these
    double sigma_f = 1.0;   // XXX Don't really know what to do with this guy, so leave it at 1.0 for now
    double sigma_f2 = sigma_f*sigma_f;
    double sigma_n = 0.01; // We expect our positions to be pretty repeatable for a given command
    double sigma_n2 = sigma_n*sigma_n;
    double length = 0.3; // We don't take a lot of samples, so look at a lot of neighbors when considering impact
    double length_2 = length*length;

    Matrix K;

    ArrayList<double[]> actual;
    ArrayList<double[]> requested;


    public ArmCalibration(String filename)
    {
        try {
            TextStructureReader fin = new TextStructureReader(new BufferedReader(new FileReader(filename)));
            int i = fin.readInt();
            requested = new ArrayList<double[]>();
            fin.blockBegin();
            while (i > 0) {
                requested.add(fin.readDoubles());
                i--;
            }
            fin.blockEnd();
            i = fin.readInt();
            actual = new ArrayList<double[]>();
            fin.blockBegin();
            while (i > 0) {
                actual.add(fin.readDoubles());
                i--;
            }
            fin.blockEnd();
            fin.close();
        } catch (IOException ioex) {
            System.err.println("ERR: Could not read in file");
            ioex.printStackTrace();
            System.exit(1);
        }

        buildK();
    }

    public ArmCalibration(ArrayList<double[]> requested,
                          ArrayList<double[]> actual)
    {
        this.requested = requested;
        this.actual = actual;

        buildK();
    }

    private void buildK()
    {
        // f(actual) = requested
        //
        // Feed in where you want to wind up to get a request
        double[][] M = new double[actual.size()][];
        for (int i = 0; i < M.length; i++) {
            M[i] = new double[actual.size()];
        }

        for (int i = 0; i < M.length; i++) {
            for (int j = 0; j < M[i].length; j++) {
                M[i][j] = kernel(actual.get(i), actual.get(j));
            }
        }

        K = (new Matrix(M)).inverse();

    }

    public void save(String filename)
    {
        try {
            TextStructureWriter fout = new TextStructureWriter(new BufferedWriter(new FileWriter(filename)));

            fout.writeInt(requested.size());
            fout.blockBegin();
            for (int i = 0; i < requested.size(); i++) {
                fout.writeDoubles(requested.get(i));
            }
            fout.blockEnd();
            fout.writeInt(actual.size());
            fout.blockBegin();
            for (int i = 0; i < actual.size(); i++) {
                fout.writeDoubles(actual.get(i));
            }
            fout.blockEnd();
            fout.close();
        } catch (IOException ioex) {
            System.err.println("ERR: Could not write to file");
            ioex.printStackTrace();
        }
    }

    private double kernel(double[] x, double[] x_)
    {
        double k = sigma_f2*Math.exp(-LinAlg.magnitude(LinAlg.subtract(x, x_))/
                                 (2*length_2));
        if (Arrays.equals(x, x_))
            k += sigma_n2;

        return k;
    }

    private double[] makeK_(double[] in)
    {
        double[] K_ = new double[actual.size()];
        for (int i = 0; i < K_.length; i++) {
            K_[i] = kernel(in, actual.get(i));
        }

        return K_;
    }

    // GP based prediction of function mapping desired position to requests
    public double[] map(double[] in)
    {
        double[] K_ = makeK_(in);
        double C = kernel(in, in);

        double[] out = new double[3];

        double[] w = K.times(K_);
        assert (w.length == requested.size());
        for (int i = 0; i < w.length; i++) {
            LinAlg.plusEquals(out, requested.get(i), w[i]);
        }

        return out;
    }
}
