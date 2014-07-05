package probcog.camera.util;

import java.io.IOException;
import java.util.*;
import javax.swing.*;
import java.awt.*;
import java.awt.image.*;
import java.awt.event.*;

import april.vis.*;
import april.jmat.*;
import april.util.*;
import april.config.*;

/** Light utility class for detecting synchronization errors for
 * libdc1394 image streams.  Setup as directed, add timestamps with
 * add__() methods, and get status by running verify and checking the
 * return value against the listed enums.
 **/
public class SyncErrorDetector
{
    public static final int SYNC_GOOD           = 1;
    public static final int SYNC_BAD            = 2;
    public static final int RECOMMEND_ACTION    = 3;

    private boolean gui;
    private SyncGUI sg;
    private double times[];         // recorded time values
    private double chi2Tolerance;   // chi^2 tolerance
    private double minimumSlope;    // minimum allowed slope for timestamp
    private double timeThresh;      // time threshold after which to recommend quitting
    private int samples;            // total samples
    private int nsamples;           // samples so far

    int lastVerification = 0;
    long timeOfFailure = Long.MAX_VALUE;
    boolean shouldQuit = false;
    int verbosity = 0;

    /** Simplified constructor.
     **/
    public SyncErrorDetector(int samples, double chi2Tolerance, double minimumSlope, double timeThresh)
    {
        this(samples, chi2Tolerance, minimumSlope, timeThresh, 0, false);
    }

    /** Config file constructor (requires input config object is the smallest child)
     **/
    public SyncErrorDetector(Config config)
    {
        this(config.requireInt("samples"),
             config.requireDouble("chi2Tolerance"),
             config.requireDouble("minimumSlope"),
             config.requireDouble("timeThresh"),
             config.getInt("verbosity", 0),
             config.getBoolean("gui", false));
    }

    public SyncErrorDetector(int samples, double chi2Tolerance, double minimumSlope,
                             double timeThresh, int verbosity, boolean useGUI)
    {
        this.samples = samples;
        this.chi2Tolerance = chi2Tolerance;
        this.minimumSlope = minimumSlope;
        this.timeThresh = timeThresh;
        this.verbosity = verbosity;
        times = new double[samples];

        this.gui = useGUI;
        if (this.gui) {
            sg = new SyncGUI();
            new Thread(sg).start();
        }
    }

    /** Add time from Point Grey image buffer (conversion from 4-byte timestamp included)
     * @param buf[] - raw byte image buffer from which to grab timestamp (from first 4 bytes)
     **/
    public void addTimePointGreyFrame(byte buf[])
    {
        int uiRawTimestamp = (buf[0]&0xFF) << 24 | (buf[1]&0xFF) << 16 | (buf[2]&0xFF) << 8 | (buf[3]&0xFF);
        int nSecond        = (uiRawTimestamp >> 25) & 0x7F;   // get rid of cycle_* - keep 7 bits
        int nCycleCount    = (uiRawTimestamp >> 12) & 0x1FFF; // get rid of offset
        int nCycleOffset   = (uiRawTimestamp >>  0) & 0xFFF;  // get rid of *_count

        double timestamp =  (double) nSecond + (((double) nCycleCount + ((double) nCycleOffset / 3072.0)) / 8000.0);
        addTime(timestamp);
    }

    /** Add timestamp as double.
     * @param newTime - time to add
     **/
    public void addTime(double newTime)
    {
        double newTimes[] = new double[times.length];
        // drop oldest value
        for (int i=0; i < times.length-1; i++)
            newTimes[i] = times[i+1];

        newTimes[times.length-1] = newTime;
        times = newTimes;

        nsamples++;
    }

    /** Verify sync lock.  Returns enum such as SYNC_GOOD or RECOMMEND_ACTION (defined above).
     **/
    public int verify()
    {
        if (shouldQuit)
            return RECOMMEND_ACTION;

        double dts[] = getDeltaTimes();
        ArrayList<double[]> xys = new ArrayList<double[]>();
        for (int i=0; i < dts.length; i++)
            xys.add(new double[] { i,
                                   dts[i]});

        double lineFit[] = LinAlg.fitLine(xys);
        if (verbosity > 1) {
            System.out.printf("DBG: SYNC: %8f:: chi^2 for derivative: %8.2f\tnsamples: %2d samples: %2d\tlastVerification: %2d timeOfFailure: %16.2f ",
                              ((double) TimeUtil.utime())/1000000.0, lineFit[2], nsamples, samples,
                              lastVerification, ((double) timeOfFailure)/1000000.0);
        }

        // initialized?
        if (nsamples < samples) {
            if (verbosity > 1)
                System.out.println("DBG: SYNC: Not initialized");
            else
                System.out.print(".");
            lastVerification = 0;
            return SYNC_BAD;
        }

        if (lastVerification == 0)
            System.out.println("\nDBG: SYNC: Stream initialized.");

        // good quality?
        if (lineFit[2] > chi2Tolerance || lineFit[1] < minimumSlope) {
            if (verbosity > 1)
                System.out.println("WRN: SYNC: Bad sync");

            // did we just change from "good" or "initializing"?
            if (lastVerification != -1) {
                timeOfFailure = TimeUtil.utime();
                if (verbosity > 0)
                    System.out.printf("WRN: SYNC: sync went bad at %.5f\n", ((double) timeOfFailure)/1000000.0);
            }

            double secSinceFailure = ((double) (TimeUtil.utime() - timeOfFailure))/1000000.0;
            if (secSinceFailure > timeThresh) {
                if (verbosity > 0)
                    System.out.printf("ERR: SYNC: Recommending quit as of now. (%.3f seconds since failure, threshold is %.3f)\n",
                                      secSinceFailure, timeThresh);
                shouldQuit = true;
            }

            lastVerification = -1;
            return SYNC_BAD;
        }

        if (verbosity > 1)
            System.out.println("DBG: SYNC: Sync ok");

        timeOfFailure = Long.MAX_VALUE;
        lastVerification = 1;
        return SYNC_GOOD;
    }

    public double[] getTimes()
    {
        return times;
    }

    public double[] getDeltaTimes()
    {
        double dts[] = new double[times.length-1];
        for (int i=1; i < times.length; i++) {
            double t1 = times[i];
            double t0 = times[i-1];

            dts[i-1] = (t1 - t0 + 128.0)%128.0;
        }

        return dts;
    }

    private class SyncGUI implements Runnable, ParameterListener
    {
        JFrame jf;
        VisWorld vw;
        VisWorld.Buffer vbtimes;
        VisWorld.Buffer vbdts;
        VisWorld.Buffer vbgraph;
        VisWorld.Buffer vbtimesfit;
        VisWorld.Buffer vbdtsfit;
        VisCanvas vc;
        ParameterGUI pg;

        private SyncGUI()
        {
            setupPG();
            setupVis();
            setupGUI();
        }

        private void setupPG()
        {
            pg = new ParameterGUI();
            pg.addDoubleSlider("seconds", "Seconds", 0, 100, 1);
            pg.addDoubleSlider("sigma", "Sigma" , 0, 100, 0.5);
            pg.addDoubleSlider("rate", "Rate", 0, 150, 10);
            pg.addButtons("simulate", "Simulate new data");
            pg.addListener(this);
        }

        private void setupVis()
        {
            vw = new VisWorld();
            VisLayer vl = new VisLayer(vw);
            vc = new VisCanvas(vl);

            vc.setBackground(Color.black);

            // Set the perspectiveness before fit2d
            VisCameraManager.CameraPosition pos = vl.cameraManager.getCameraTarget();
            pos.perspectiveness = 1;
            vl.cameraManager.goUI(pos);


            vl.cameraManager.fit2D(new double[] {-0.1, -0.1},
                                   new double[] { 2.1,  1.1}, true);


            vbgraph = vw.getBuffer("Graph");
            vbtimes = vw.getBuffer("Times");
            vbdts = vw.getBuffer("DTs");
            vbtimesfit = vw.getBuffer("Times Fit");
            vbdtsfit = vw.getBuffer("DTs Fit");
        }

        private void setupGUI()
        {
            jf = new JFrame("Sync Error Detector for libdc1394");
            jf.setLayout(new BorderLayout());
            jf.add(vc, BorderLayout.CENTER);
            jf.add(pg, BorderLayout.SOUTH);

            jf.setSize(600,500);
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setVisible(true);
        }

        public void parameterChanged(ParameterGUI pg, String name)
        {
            if (name.equals("simulate")) {
                double seconds = pg.gd("seconds");
                double sigma = pg.gd("sigma");
                new SimThread(seconds, sigma).start();
            }
        }

        public void run()
        {
            while (true)
            {
                TimeUtil.sleep(100);

                redraw();
            }
        }

        void redraw()
        {
            double ed_times[] = getTimes();
            double ed_dts[] = getDeltaTimes();

            if (ed_times == null || ed_dts == null)
                return;

            double minTime = 0;
            double maxTime = 128;
            double timeRange = maxTime - minTime;

            double minDT = 0;
            double maxDT = 1;
            double dtRange = maxDT - minDT;

            {
                vbgraph.setDrawOrder(-100);
                vbgraph.addBack(new VisChain(LinAlg.translate(.5,.5,0),
                                             new VzRectangle(1,1,
                                                             new VzMesh.Style(new Color(100, 100, 100, 100)))));
                vbgraph.addBack(new VisChain(LinAlg.translate(0, 1, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                                                        "<<blue>>Times")));
                vbgraph.addBack(new VisChain(LinAlg.translate(1, 1, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.BOTTOM_LEFT,
                                                        "<<cyan>>DTs")));

                vbgraph.addBack(new VisChain(LinAlg.translate(0, 0, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                                                        Double.toString(minTime))));
                vbgraph.addBack(new VisChain(LinAlg.translate(0, 1, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_RIGHT,
                                                        Double.toString(maxTime))));

                vbgraph.addBack(new VisChain(LinAlg.translate(1, 0, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.BOTTOM_LEFT,
                                                        Double.toString(minDT))));
                vbgraph.addBack(new VisChain(LinAlg.translate(1, 1, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_LEFT,
                                                        Double.toString(maxDT))));

                vbgraph.addBack(new VisChain(LinAlg.translate(0, 0, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_LEFT,
                                                        Double.toString(0))));
                vbgraph.addBack(new VisChain(LinAlg.translate(1, 0, 0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_RIGHT,
                                                        Double.toString(ed_times.length))));
            }

            {
                vbdts.setDrawOrder(-50);
                ArrayList<double[]> xys = new ArrayList<double[]>();
                for (int i=0; i < ed_dts.length; i++)
                    xys.add(new double[] { (double) (i+1) / ed_times.length,
                                           ((double) (ed_dts[i]-minDT)/dtRange) });

                vbdts.addBack(new VzLines(new VisVertexData(xys), VzLines.LINE_STRIP,
                                           new VzLines.Style(Color.cyan, 1)));
            }

            {
                vbtimes.setDrawOrder(-1);
                ArrayList<double[]> xys = new ArrayList<double[]>();
                for (int i=0; i < ed_times.length; i++)
                    xys.add(new double[] {  (double) i / ed_times.length,
                                            ((double) (ed_times[i]-minTime)/timeRange) });

                vbtimes.addBack(new VzLines(new VisVertexData(xys), VzLines.LINE_STRIP,
                                             new VzLines.Style(Color.blue, 3)));
            }

            {
                ArrayList<double[]> xys = new ArrayList<double[]>();
                for (int i=0; i < ed_dts.length; i++)
                    xys.add(new double[] { i,
                                           ed_dts[i]});

                // line fitting
                double lineFit[] = LinAlg.fitLine(xys);
                ArrayList<double[]> line = new ArrayList<double[]>();

                line.add(new double[] {xys.get(0)[0]                                                / ed_dts.length,
                                       ((lineFit[0]*xys.get(0)[0] + lineFit[1])            - minDT) / dtRange });

                line.add(new double[] {xys.get(xys.size()-1)[0]                                     / ed_dts.length,
                                       ((lineFit[0]*xys.get(xys.size()-1)[0] + lineFit[1]) - minDT) / dtRange });

                vbdtsfit.setDrawOrder(20);
                vbdtsfit.addBack(new VzLines(new VisVertexData(line), VzLines.LINE_STRIP,
                                              new VzLines.Style(Color.yellow, 2)));
                vbgraph.addBack(new VisChain(LinAlg.translate(1.5, 0.7,0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(
                                                 VzText.ANCHOR.TOP_LEFT,
                                                 String.format("<<yellow>>DTs:   %10.3f", lineFit[0]))));
                vbgraph.addBack(new VisChain(LinAlg.translate(1.5, 0.4,0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_LEFT,
                                                        String.format("<<yellow>>DTs:   %10.3f", lineFit[1]))));
                vbgraph.addBack(new VisChain(LinAlg.translate(1.5,0.1,0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_LEFT,
                                                        String.format("<<yellow>>DTs:   %10.3f", lineFit[2]))));
            }

            {
                ArrayList<double[]> xys = new ArrayList<double[]>();
                for (int i=0; i < ed_times.length; i++)
                    xys.add(new double[] { i,
                                           ed_times[i]});

                // line fitting
                double lineFit[] = LinAlg.fitLine(xys);
                ArrayList<double[]> line = new ArrayList<double[]>();

                line.add(new double[] {xys.get(0)[0]                                                  / ed_times.length,
                                       ((lineFit[0]*xys.get(0)[0] + lineFit[1])            - minTime) / timeRange });

                line.add(new double[] {xys.get(xys.size()-1)[0]                                       / ed_times.length,
                                       ((lineFit[0]*xys.get(xys.size()-1)[0] + lineFit[1]) - minTime) / timeRange });

                vbtimesfit.setDrawOrder(10);
                vbtimesfit.addBack(new VzLines(new VisVertexData(line),VzLines.LINE_STRIP,
                                                new VzLines.Style(Color.red, 2)));
                vbgraph.addBack(new VisChain(LinAlg.translate(1.5,0.8,0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_LEFT,
                                                        String.format("<<red>>Times: %10.3f", lineFit[0]))));
                vbgraph.addBack(new VisChain(LinAlg.translate(1.5, 0.5,0),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_LEFT,
                                                        String.format("<<red>>Times: %10.3f", lineFit[1]))));
                vbgraph.addBack(new VisChain(LinAlg.translate(1.5, 0.2, 0 ),
                                             LinAlg.scale(1/8.0),
                                             new VzText(VzText.ANCHOR.TOP_LEFT,
                                                        String.format("<<red>>Times: %10.3f", lineFit[2]))));
            }

            vbgraph.addBack(new VisChain(LinAlg.translate(1.5,0.9,0),
                                         LinAlg.scale(1/8.0),
                                         new VzText(VzText.ANCHOR.TOP_LEFT,
                                                    String.format("Slopes (M)"))));
            vbgraph.addBack(new VisChain(LinAlg.translate(1.5,0.6,0),
                                         LinAlg.scale(1/8.0),
                                         new VzText(VzText.ANCHOR.TOP_LEFT,
                                                    String.format("Offsets (b)"))));
            vbgraph.addBack(new VisChain(LinAlg.translate(1.5,0.3,0),
                                         LinAlg.scale(1/8.0),
                                         new VzText(VzText.ANCHOR.TOP_LEFT,
                                                    String.format("Chi^2 Errors"))));

            {
                int res = verify();
                if (res == SYNC_BAD)
                    vbgraph.addBack(new VisChain(LinAlg.translate(1.5,1.5,0),
                                                 LinAlg.scale(1/8.0),
                                                 new VzText(VzText.ANCHOR.TOP_LEFT,
                                                            String.format("<<yellow>>Sync Bad!"))));
                else if (res == RECOMMEND_ACTION)
                    vbgraph.addBack(new VisChain(LinAlg.translate(1.5,1.5,0),
                                         LinAlg.scale(1/8.0),
                                                 new VzText(VzText.ANCHOR.TOP_LEFT,
                                                            String.format("<<red>>Action recommended!"))));
                else
                    vbgraph.addBack(new VisChain(LinAlg.translate(1.5,1.5,0),
                                         LinAlg.scale(1/8.0),
                                                 new VzText(VzText.ANCHOR.TOP_LEFT,
                                                            String.format("<<blue>>Sync good"))));
            }

            vbgraph.swap();
            vbdts.swap();
            vbtimes.swap();
            vbtimesfit.swap();
            vbdtsfit.swap();
        }

        public class SimThread extends Thread
        {
            double seconds;
            double sigma;

            public SimThread(double seconds, double sigma)
            {
                this.seconds = seconds;
                this.sigma = sigma;
            }

            public void run()
            {
                double rate = pg.gd("rate");
                double last = times[times.length-1];
                long t0 = TimeUtil.utime();
                Random r = new Random();
                while (TimeUtil.utime() - t0 < this.seconds*1000000)
                {
                    int ms = Math.max(0, (int) (1.0/rate*1000 + r.nextGaussian()*sigma));

                    TimeUtil.sleep(ms);

                    double val = (TimeUtil.utime() - t0)/1000000.0 + last;
                    SyncErrorDetector.this.addTime(val);
                    SyncErrorDetector.this.verify();
                }
            }
        }
    }

    public static void main(String args[])
    {
        Config config = ConfigUtil.getDefaultConfig(args);
        SyncErrorDetector sed = new SyncErrorDetector(config.getChild("camera.sync"));
    }
}
