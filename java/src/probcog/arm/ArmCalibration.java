package probcog.arm;

import java.awt.*;
import java.io.*;
import javax.swing.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.util.*;
import april.vis.*;

import probcog.gui.*;

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
    double length = 0.5; // We don't take a lot of samples, so look at a lot of neighbors when considering impact
    double length_2 = length*length;

    Matrix K;

    ArrayList<double[]> actual = null;
    ArrayList<double[]> requested = null;


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
        if (actual == null || requested == null)
            return;

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

    /** Evaluate the calibration */
    public void validate()
    {
        double err2Actual = 0;
        double err2Calib = 0;
        for (int i = 0; i < requested.size(); i++) {
            double[] req = requested.get(i);
            double[] act = actual.get(i);
            double[] cal = map(requested.get(i));
            err2Actual += LinAlg.magnitude(LinAlg.subtract(req, act));
            err2Calib += LinAlg.magnitude(LinAlg.subtract(req, cal));
        }


        // Validate on novel positions
        /*double err2Novel = 0;
        int examples = 0;
        for (double x = -.3; x <= .3; x += 0.02) {
            for (double y = .1; y <= .3; y+= 0.02) {
                for (double z = 0.05; z <= 0.15; z += 0.02) {
                    double[] xyz_r = new double[] {x,y,z};
                    double[] xyz_c = map(xyz_r);
                    err2Novel += LinAlg.magnitude(LinAlg.subtract(xyz_r, xyz_c));
                    examples++;
                }
            }
        }
        System.out.println();
        System.out.println("Initial Data Validation results");
        System.out.println("===============================");
        System.out.printf( "err^2 pre-calibration:  %.5f\n", err2Actual/requested.size());
        System.out.printf( "err^2 post-calibration: %.5f\n", err2Calib/requested.size());
        System.out.println();
        System.out.println("Novel Data Validation results");
        System.out.println("===============================");
        System.out.printf( "err^2 post-calibration: %.5f\n", err2Novel/examples);
        */
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
        if (K == null)
            return in;

        double[] K_ = makeK_(in);
        double C = kernel(in, in);

        // f(desired) = error, where desired maps to the actual results
        // from testing. We want to determine what command to ACTUALLY
        // issue based on our desired end location. This is determined
        // by seeing where arm actually went for given (requested)
        // commands.
        //
        // The actual position we will command the arm to based on our
        // desired endpoint is:
        //
        // issued = desired + error
        //
        // where error denotes some correction term based on our past
        // experiences. In this case, error is the difference between
        // the end position and the command that will get it there
        // (or actual - requested)
        double[] out = new double[in.length];

        double[] w = K.times(K_);
        assert (w.length == requested.size());
        for (int i = 0; i < w.length; i++) {
            double[] diff = LinAlg.subtract(requested.get(i),
                                            actual.get(i));
            //double[] diff = LinAlg.subtract(actual.get(i),
            //                                requested.get(i));
            LinAlg.plusEquals(out,
                              diff,
                              w[i]);
        }

        out = LinAlg.add(in, out);
        System.out.printf("||w||=%f\n",LinAlg.magnitude(w));
        for (int j = 0; j < out.length; j++) {
            System.out.printf("%f, ",in[j]);
        }
        System.out.printf(" --> ");
        for (int j = 0; j < out.length; j++) {
            System.out.printf("%f, ",out[j]);
        }
        System.out.println();


        return out;
    }

    static public class CalibrationListener implements ParameterListener
    {
        VisWorld vw;
        ArmCalibration ac;

        public CalibrationListener(ArmCalibration ac, VisWorld vw)
        {
            this.ac = ac;
            this.vw = vw;
        }

        public void parameterChanged(ParameterGUI pg, String name)
        {
            if (name.equals("enableSlider")) {
                pg.setEnabled("z", pg.gb("enableSlider"));
            }
            if (name.equals("relativeMap")) {
                pg.setEnabled("map-max", !pg.gb("relativeMap"));
            }
            if (pg.gb("enableSlider")) {
                // Slice sample
                renderRange(pg.gd("z"), pg.gd("z"), pg.gd("z-step"),
                            .1, .4, pg.gd("r-step"),
                            0, Math.toRadians(360), Math.toRadians(pg.gi("t-step")),
                            pg.gb("relativeMap"), pg.gd("map-max"));
            } else {
                // Full sampling
                renderRange(0, .3, pg.gd("z-step"),
                            .1, .4, pg.gd("r-step"),
                            0, Math.toRadians(360), Math.toRadians(pg.gi("t-step")),
                            pg.gb("relativeMap"), pg.gd("map-max"));
            }
        }

        /** Renders the specified range of values as a heatmap based on
         *  the difference between the requested position and the mapped
         *  position.
         *
         *  No knowledge of the actual results provided.
         **/
        private void renderRange(double minZ, double maxZ, double zstep,
                                 double minR, double maxR, double rstep,
                                 double minT, double maxT, double tstep,
                                 boolean useRelativeMapping, double max)
        {
            // Color map generated with HeatMapGenerator
            // Lower error == blue
            // Higher error == orange
            // Middling == purple
            int[] map = {
                0xff0000,
                0x0066ff};

            ArrayList<double[]> points = new ArrayList<double[]>();
            ArrayList<double[]> errors = new ArrayList<double[]>();
            double minErr = Double.POSITIVE_INFINITY;
            double maxErr = Double.NEGATIVE_INFINITY;
            for (double t = minT; t <= maxT; t += tstep) {
                for (double r = minR; r <= maxR; r += rstep) {
                    for (double z = minZ; z <= maxZ; z += zstep) {
                        double[] xyz_r = new double[] {r*Math.cos(t),
                                                       r*Math.sin(t),
                                                       z};
                        double[] xyz_c = ac.map(xyz_r);

                        double[] xyze = LinAlg.resize(xyz_r, 4);
                        xyze[3] = LinAlg.magnitude(LinAlg.subtract(xyz_r,
                                                                   xyz_c));
                        minErr = Math.min(xyze[3], minErr);
                        maxErr = Math.max(xyze[3], maxErr);

                        points.add(xyz_r);
                        errors.add(xyze);
                    }
                }
            }

            ColorMapper colorMapper;
            if (useRelativeMapping)
                colorMapper = new ColorMapper(map, minErr, maxErr);
            else
                colorMapper = new ColorMapper(map, 0, max);
            VisColorData vcd = colorMapper.makeColorData(errors, 3);    // XXX

            VisWorld.Buffer vb = vw.getBuffer("errorMap");
            vb.addBack(new VisLighting(false,
                                       new VzPoints(new VisVertexData(points),
                                                    new VzPoints.Style(vcd, 2))));
            vb.swap();
        }
    }

    /** Validation/make figures */
    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h',"help",false,"Show this help screen");
        opts.addString('w',"world",null,"Sim world");
        opts.addString('a',"arm",null,"Arm calibration");

        if (!opts.parse(args)) {
            System.err.println("ERR: Error parsing args - "+opts.getReason());
            System.exit(1);
        }

        if (opts.getBoolean("help"))
        {
            opts.doHelp();
            System.exit(0);
        }

        JFrame jf = new JFrame("Calibration Visualizer");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        assert (opts.getString("arm") != null);
        ArmCalibration calibration = new ArmCalibration(opts.getString("arm"));
        ProbCogSimulator simulator = new ProbCogSimulator(opts, vw, vl, vc);

        ParameterGUI pg = new ParameterGUI();
        pg.addBoolean("relativeMap", "Relative color mapping", false);
        pg.addBoolean("enableSlider", "Enable Slider", false);
        pg.addDoubleSlider("map-max", "Map Max", .001, 0.2, 0.05);
        pg.addDoubleSlider("z-step", "Z step", .01, .1, 0.025);
        pg.addIntSlider("t-step", "Theta step", 1, 30, 15);
        pg.addDoubleSlider("r-step", "R step", .01, .1, 0.025);
        pg.addDoubleSlider("z", "Z-level", 0, .3, 0);
        pg.setEnabled("z", false);
        pg.addListener(new CalibrationListener(calibration, vw));
        jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }
}
