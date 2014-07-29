package probcog.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.vis.*;
import april.util.*;
import april.lcmtypes.*;

import probcog.vis.*;
import probcog.lcmtypes.*;
import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;

public class MonteCarloBot implements SimObject
{
    Random r = new Random();

    int ROBOT_ID = 6;
    SimWorld sw;

    Object runLock = new Object();
    boolean running = false;

    FastDrive drive;
    ArrayList<double[]> trajectoryTruth = new ArrayList<double[]>();
    ArrayList<double[]> trajectoryOdom = new ArrayList<double[]>();

    FollowWall law;
    ClassificationCounterTest test;

    public MonteCarloBot(SimWorld sw)
    {
        this.sw = sw;
    }

    // === Random sampling interface =========================
    public void init(FollowWall law, ClassificationCounterTest test, double[] xyt)
    {
        this.law = law;
        this.test = test;

        drive = new FastDrive(sw, this, xyt);
        drive.centerOfRotation = new double[] { 0.13, 0, 0 };

        trajectoryTruth.add(drive.poseTruth.pos);
        trajectoryOdom.add(drive.poseOdom.pos);

    }

    // Simulate all steps, keeping track of trajectory, etc.
    public void simulate()
    {
        // Precalculated paramters
        HashSet<SimObject> ignore = new HashSet<SimObject>();
        ignore.add(this);
        double radstep = Math.toRadians(5);
        double minDeg = -135;
        double maxDeg = 135;
        double maxRange = 29.9;
        double rad0 = Math.toRadians(minDeg);
        double rad1 = Math.toRadians(maxDeg);
        laser_t laser = new laser_t();
        laser.utime = TimeUtil.utime();

        // While control law has not finished OR timeout, try updating
        int timeout = (int)(30.0/FastDrive.DT);
        Tic tic = new Tic();
        double time = 0;
        while (!test.conditionMet() && timeout > 0) {
            tic.tic();
            // LASER UPDATE
            double[][] T_truth = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                                                        drive.poseTruth.pos),
                                                 LinAlg.translate(0.3, 0, 0.25));
            double[][] T_odom = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseOdom.orientation,
                                                                       drive.poseOdom.pos),
                                                LinAlg.translate(0.3, 0, 0.25));

            double ranges[] = Sensors.laser(sw, ignore, T_truth, (int) ((rad1-rad0)/radstep), rad0, radstep, maxRange);

            double mean = 0;
            double stddev = 0.01;   // XXX Laser noise
            for (int i = 0; i < ranges.length; i++) {
                if (ranges[i] >= maxRange || ranges[i] < 0)
                    continue;
                ranges[i] = Math.min(maxRange, ranges[i]+r.nextGaussian()*stddev*ranges[i]);
            }

            laser.nranges = ranges.length;
            laser.ranges = LinAlg.copyFloats(ranges);
            laser.rad0 = (float) rad0;
            laser.radstep = (float) radstep;

            // DRIVE UPDATE
            law.init(laser);
            diff_drive_t dd = law.drive(laser, FastDrive.DT);
            drive.motorCommands[0] = dd.left;
            drive.motorCommands[1] = dd.right;
            drive.update();

            // CHECK CLASSIFICATIONS
            // XXX

            laser.utime += FastDrive.DT*1000000;
            trajectoryTruth.add(drive.poseTruth.pos);
            trajectoryOdom.add(drive.poseOdom.pos);

            // TIME UPDATE
            timeout--;
            time += tic.toc();
            //System.out.printf("\t%f [s]\n", tic.toc());
        }
        System.out.printf("%f [s]\n", time);
    }

    // === SimObject interface ===============================
    public double[][] getPose()
    {
        return LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                      drive.poseTruth.pos);
    }

    public void setPose(double[][] T)
    {
        drive.poseTruth.orientation = LinAlg.matrixToQuat(T);
        drive.poseTruth.pos = new double[] { T[0][3], T[1][3], 0 };
    }

    static final Shape shape = new SphereShape(0.4);
    public Shape getShape()
    {
        return shape;
    }

    static Model4 model4 = new Model4(null, Color.blue, 0.5);
    public VisObject getVisObject()
    {
        VisChain vc = new VisChain(new VzLines(new VisVertexData(trajectoryTruth),
                                               VzLines.LINE_STRIP,
                                               new VzLines.Style(Color.blue, 2)),
                                   new VzLines(new VisVertexData(trajectoryOdom),
                                               VzLines.LINE_STRIP,
                                               new VzLines.Style(Color.red, 2)),
                                   getPose(),
                                   model4);
        return vc;
    }

    public void read(StructureReader ins) throws IOException
    {
        // XXX
    }

    public void write(StructureWriter outs) throws IOException
    {
        // XXX
    }

    public void setRunning(boolean run)
    {
        synchronized (runLock) {
            this.running = run;
        }
    }
}
