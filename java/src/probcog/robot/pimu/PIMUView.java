package magic.pimu;

import java.awt.*;
import java.util.*;
import java.io.*;

import javax.swing.*;

import april.jserial.*;
import april.vis.*;
import april.jmat.*;
import april.util.*;

import magic.lcmtypes.*;
import magic.util.*;

import lcm.lcm.*;

public class PIMUView implements PIMU.PIMUListener, ParameterListener, LCMSubscriber
{
    JFrame jf;
    VisWorld vw = new VisWorld();
    VisLayer vl = new VisLayer(vw);
    VisCanvas vc = new VisCanvas(vl);
    ParameterGUI pg;

    VisObject pcb = new VisChain(LinAlg.scale(0.0254, 0.0254, 0.0254),
                                 new VzBox(2.0, 0.7, 0.062, new VzMesh.Style(Color.green)),
                                 LinAlg.translate(-0.75, -0.2, 0.15),
                                 new VzBox(0.49, 0.29, 0.29, new VzMesh.Style(Color.gray)));

    PIMU pimu;

    JWaveform integratorPlots[];

    JWaveform offsetPlot;

    LCM lcm = LCM.getSingleton();

    int integratorIndices[] = new int[] { 0, 4, 3, 7 };
    String integratorNames[] = new String[] { "Yaw 1", "Yaw 2", "Roll", "Pitch" };
    Color integratorColors[] = new Color[] { Color.yellow, Color.green, Color.red, Color.cyan };

    ArrayList<pimu_t> log = null; // null means not logging
    ArrayList<ArrayList<pimu_t>> logs = new ArrayList<ArrayList<pimu_t>>();

    PIMUFilter filter = new PIMUFilter();

    GetOpt gopt;

    public PIMUView(PIMU pimu, GetOpt gopt)
    {
        this.pimu = pimu;
        this.gopt = gopt;

        pg = new ParameterGUI();
        pg.addDoubleSlider("maxintperiod", "Acceleration integration period (s)", 0.5, 30, filter.maxintperiod);
        pg.addDoubleSlider("maxv", "Maximum vehicle velocity (m/s)", 1, 10, filter.maxv);
        pg.addCheckBoxes("gyro-enable", "Enable Gyro Calibration", true,
                         "accel-enable", "Enable Accelerometer orientation", true);
        pg.addButtons("reset", "reset rpy=0",
                      "stats", "Get PIMU stats");

        if (gopt.getBoolean("enable-calibration")) {
            pg.addButtons("params", "Read Params",
                          "startstop", "Start/Stop recording",
                          "write", "Write parameters",
                          "reset-param", "Reset to default parameters");
        }

        pg.addListener(this);

        JPanel integratorPanel = new JPanel();
        integratorPanel.setLayout(new GridLayout(5, 1));
        integratorPlots = new JWaveform[8];

        for (int i = 0; i < integratorIndices.length; i++) {
            integratorPlots[i] = new JWaveform(new double[] { .5, 1, 2, 5, 10, 30, 60, 240 }, 5, 4,
                                               new double[] { .1, .25, .4, 1, 2, 4, 10, 30, 90, 180, 360, 720 }, 9, 4);
            JPanel jp = new JPanel();
            jp.setBorder(BorderFactory.createTitledBorder(integratorNames[i]+" deg"));
            jp.setLayout(new BorderLayout());
            jp.add(integratorPlots[i]);
            integratorPanel.add(jp);
        }

        offsetPlot = new JWaveform(new double[] { .5, 1, 2, 5, 10, 30, 60, 240 }, 5, 4,
                                   new double[] { 0.01, 0.1, 0.2, 0.5, 1.0, 2.0, 5.0, 10.0 }, 3, 4);
        if (true) {
            JPanel jp = new JPanel();
            jp.setBorder(BorderFactory.createTitledBorder("Gyro Offsets (deg/s)"));
            jp.setLayout(new BorderLayout());
            jp.add(offsetPlot);
            integratorPanel.add(jp);
        }

        for (int i = 0; i < integratorIndices.length; i++) {
            integratorPlots[i].setColor(integratorNames[i]+" cal", integratorColors[i]);
            integratorPlots[i].setColor(integratorNames[i]+" raw", integratorColors[i]);

            integratorPlots[i].setWidth(integratorNames[i]+" cal", 2.0f);
            integratorPlots[i].setWidth(integratorNames[i]+" raw", 2.0f);

            offsetPlot.setColor(integratorNames[i], integratorColors[i]);

            for (int j = 0; j < integratorIndices.length; j++) {
                integratorPlots[i].addViewBuddy(integratorPlots[j], true, true);
                offsetPlot.addViewBuddy(integratorPlots[j], true, false);
                integratorPlots[j].addViewBuddy(offsetPlot, true, false);
            }
        }

        jf = new JFrame("PIMUView");
        jf.setLayout(new BorderLayout());
        JSplitPane jsp = new JSplitPane(JSplitPane.HORIZONTAL_SPLIT, integratorPanel, vc);
        jsp.setDividerLocation(0.5);
        jsp.setResizeWeight(0.5);

        jf.add(jsp, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setSize(800,600);
        jf.setVisible(true);

        vl.cameraManager.uiLookAt(new double[] { -.07735, -.18521, .13798 },
                                  new double[] { 0.02186, 0.06585, 0 },
                                  new double[] { 0.16726, .42328, 0.89043 },
                                  true);

        VzGrid.addGrid(vw);

        if (gopt.getString("device").equals("lcm"))
            lcm.subscribe("PIMU", this);
        else
            pimu.addListener(this);
    }

    public synchronized void parameterChanged(ParameterGUI pg, String name)
    {
        filter.maxintperiod = pg.gd("maxintperiod");
        filter.maxv = pg.gd("maxv");
        filter.enableGyroCal = pg.gb("gyro-enable");
        filter.enableAccelerometers = pg.gb("accel-enable");

        if (name.equals("reset"))
            filter.setQuaternion(LinAlg.rollPitchYawToQuat(new double[3]));

        if (name.equals("stats")) {
            pimu.doCommand(new byte[] { 30, 0 }, 500);
        }

        if (name.equals("params")) {
            PIMU.Params params = pimu.getParams();
            params.print();
        }

        if (name.equals("calibrate-offset")) {
            PIMU.Params params = pimu.getParams();

            pimu_t pimu0 = pimu.getLast();
            TimeUtil.sleep(2000);
            pimu_t pimu1 = pimu.getLast();

            for (int i = 0; i < 8; i++) {
                long usecs = (pimu1.utime - pimu0.utime);
                double err = pimu1.integrator[i] - pimu0.integrator[i]; // (in ticks*msec)
                double offset = 1000.0 * (err / usecs / params.gyro_scale[i]);

                params.gyro_offset[i] += offset;
            }

            params.print();
            pimu.setParams(params);

        }

        if (name.equals("startstop")) {

            if (log == null) {
                // begin log
                log = new ArrayList<pimu_t>();
            } else {
                // save log.
                logs.add(log);
                log = null;
                vw.getBuffer("logstat").swap();

                optimize();
            }
        }

        if (name.equals("write")) {
            PIMU.Params params = pimu.getParams();

            params.print();

            for (int i = 0; i < 8; i++) {
                params.gyro_offset[i] += filter.integrator_offset[i] * 1000.0 / params.gyro_scale[i];
            }

            for (int i = 0; i < 8; i++)
                params.gyro_scale[i] *= filter.integrator_scale[i];

            params.print();
            pimu.setParams(params);

            // we just made the PIMU do this work... the filter doesn't need to any more.
            for (int i = 0; i < 8; i++) {
                filter.integrator_scale[i] = 1.0;
                filter.integrator_offset[i] = 0;
            }

            // our old logs now have miscalibrated data
            logs.clear();
        }

        if (name.equals("reset-param")) {
            PIMU.Params params = pimu.getParams();
            params.start_signature = 0;
            pimu.setParams(params);

            // our old logs now have miscalibrated data
            logs.clear();
        }
    }

    void optimize()
    {
        // do optimization.
        PIMUFilter pf = filter.copy();
        pf.enableAccelerometers = false;
        pf.enableGyroCal = false;

        int maxiters = 100000;
        double eps = 0.005;
        double mineps = 0.000001;

//        double p[] = new double[] { 1, 1, 1, 1, 0, 0, 0, 0 };
        double p[] = new double[] { 1, 1, 1, 1 };

        for (int iter = 0; iter < maxiters; iter++) {
            int bestidx = -1;
            double besteps = 0;
            double bestScore = evaluate(pf, logs, p);

            for (int j = 0; j < p.length; j++) {
                double v = p[j];

                p[j] = v - eps;
                double score = evaluate(pf, logs, p);
                if (score < bestScore) {
                    bestScore = score;
                    bestidx = j;
                    besteps = -eps;
                }

                p[j] = v + eps;
                score = evaluate(pf, logs, p);
                if (score < bestScore) {
                    bestScore = score;
                    bestidx = j;
                    besteps = eps;
                }

                p[j] = v;
            }

            if (bestidx < 0) {
                eps /= 2;
                if (eps < mineps)
                    break;
            } else {
                p[bestidx] += besteps;
            }

//            System.out.printf("%8d : score = %15f, params = [%15f %15f %15f %15f %15f %15f %15f %15f]\n", iter, bestScore, p[0], p[1], p[2], p[3], p[4], p[5], p[6], p[7]);
            System.out.printf("%8d : score = %15f, params = [%15f %15f %15f %15f]\n", iter, bestScore, p[0], p[1], p[2], p[3]);
        }

        pf = filter;
        pf.integrator_scale[0] = p[0];
        pf.integrator_scale[3] = p[1];
        pf.integrator_scale[4] = p[2];
        pf.integrator_scale[7] = p[3];
/*
  pf.integrator_offset[0] = p[4];
  pf.integrator_offset[3] = p[5];
  pf.integrator_offset[4] = p[6];
  pf.integrator_offset[7] = p[7];
*/
    }

    static double evaluate(PIMUFilter pf, ArrayList<ArrayList<pimu_t>> logs, double p[])
    {
        pf.integrator_scale[0] = p[0];
        pf.integrator_scale[3] = p[1];
        pf.integrator_scale[4] = p[2];
        pf.integrator_scale[7] = p[3];

/*
  pf.integrator_offset[0] = p[4];
  pf.integrator_offset[3] = p[5];
  pf.integrator_offset[4] = p[6];
  pf.integrator_offset[7] = p[7];
*/
        double err = 0;

        for (int i = 0; i < 3; i++) {
            pf.useYawGyros = i;

            pf.setQuaternion(LinAlg.rollPitchYawToQuat(new double[3]));

            for (ArrayList<pimu_t> log : logs) {
                for (pimu_t d : log) {
                    pf.update(d);
                }

                double rpy[] = LinAlg.quatToRollPitchYaw(pf.getQuaternion());

                err += 3282.8*LinAlg.magnitude(rpy); // convert to degrees squared.
            }
        }

        return err / logs.size() / 3;
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            pimu_t pimu = new pimu_t(ins);
            pimuData(pimu);
        } catch (IOException ex) {
        }
    }

    public synchronized void pimuData(pimu_t pimu)
    {
        filter.update(pimu);

        if (log != null) {
            if (log.size() % 10 == 0) {
                VisWorld.Buffer vb = vw.getBuffer("logstat");
                vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.TOP_RIGHT,
                                                   new VzText(VzText.ANCHOR.TOP_RIGHT_ROUND,
                                                              String.format("<<cyan,sansserif-20>>Log size %d\n", log.size()))));
                vb.swap();
            }

            log.add(pimu);
        }

        for (int i = 0; i < integratorIndices.length; i++) {
            int idx = integratorIndices[i];
            integratorPlots[i].addData(integratorNames[i]+" cal", new double[] { pimu.utime / 1.0E6, filter.integrator_corrected[idx] / 1.0E6 }, 0.1);
            integratorPlots[i].addData(integratorNames[i]+" raw", new double[] { pimu.utime / 1.0E6, pimu.integrator[idx] / 1.0E6 }, 0.1);
            offsetPlot.addData(integratorNames[i], new double[] {pimu.utime / 1.0E6, filter.integrator_offset[idx] }, 0.1);
        }

//        lcm.publish("PIMU", pimu);


        VisWorld.Buffer vb = vw.getBuffer("accel");
        vb.addBack(new VisChain(filter.getMatrix(),
                                pcb));


        if (true) {
            // draw a cone in the direction of our observed acceleration
            // vector. Gravity must be somewhere inside it.

            double acceldir[] = filter.getAccelUp();

            // find rotation matrix that makes the z axis point in the acceldir direction
            double q[] = LinAlg.quatCompute(new double[] { 0, 0, 1}, acceldir);

            vb.addBack(new VisChain(LinAlg.scale(.1, .1, .1),
                                    new VzLines(new VisVertexData(new double[] {0,0,0},
                                                                  new double[] {0,0,1}),
                                                VzLines.LINES,
                                                new VzLines.Style(Color.blue, 2))));

            vb.addBack(new VisChain(filter.getMatrix(),
                                    LinAlg.scale(.1, .1, .1),
                                    new VzLines(new VisVertexData(new double[] {0,0,0},
                                                                  acceldir),
                                                VzLines.LINES,
                                                new VzLines.Style(Color.black, 2)),
                                    LinAlg.quatToMatrix(q),
                                    LinAlg.translate(0,0,1),
                                    LinAlg.rotateX(Math.PI),
                                    new VzCone(Math.sin(filter.getUpUncertainty()),
                                               1,
                                               0, // No VzCone.BOTTOM
                                               new VzMesh.Style(new Color(128,128,128,128)))));
        }

        vb.addBack(new VisChain(filter.getMatrix(),
                                LinAlg.scale(.0001, .0001, .0001),
                                new VzLines(new VisVertexData(new double[] {0,0,0},
                                                              new double[] { pimu.mag[0],
                                                                             pimu.mag[1],
                                                                             pimu.mag[2] }),
                                            VzLines.LINES,
                                            new VzLines.Style(Color.red, 4))));

        double rpy[] = LinAlg.quatToRollPitchYaw(filter.getQuaternion());

        vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_RIGHT,
                                           new VzText(VzText.ANCHOR.BOTTOM_RIGHT_ROUND,
                                                      String.format("<<right>>rpy deg = [ %10.3f %10.3f %10.3f ]",
                                                                    Math.toDegrees(rpy[0]), Math.toDegrees(rpy[1]), Math.toDegrees(rpy[2])))));

        vb.swap();
    }

    public static void main(String args[])
    {
        GetOpt gopt = new GetOpt();
        gopt.addInt('\0', "hz", 100, "Publish rate (Hz). Use 0 for all.");
        gopt.addString('d', "device", "/dev/pimu", "Device path, or lcm (not all features supported for lcm)");
        gopt.addBoolean('h', "help", false, "Show this help");
        gopt.addBoolean('e', "enable-calibration", false, "Enable calibration options (experts only)");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            gopt.doHelp();
            System.exit(-1);
        }

        PIMU imu = null;
        if (!gopt.getString("device").equals("lcm"))
            imu = new PIMU(gopt);

        PIMUView pt = new PIMUView(imu, gopt);
    }
}
