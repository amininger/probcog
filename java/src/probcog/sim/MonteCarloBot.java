package probcog.sim;

import java.io.*;
import java.util.*;

import april.jmat.*;
import april.sim.*;
import april.vis.*;
import april.util.*;
import april.lcmtypes.*;

import probcog.vis.*;
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

    // Parameters
    double gridmap_meters_per_pixel = 0.1;
    double gridmap_range = 10;

    public MonteCarloBot(SimWorld sw)
    {
        this.sw = sw;
    }

    // === Random sampling interface =========================
    public void init(FollowWall law, ClassificationCounterTest test, double[] xyt)
    {
        this.law = law;
        this.test = test;
        trajectoryTruth.add(drive.poseTruth.pos);
        trajectoryOdom.add(drive.poseOdom.pos);

        drive = new FastDrive(sw, this, xyt);
        drive.centerOfRotation = new double[] { 0.13, 0, 0 };
    }

    // Simulate all steps, keeping track of trajectory, etc.
    public void simulate()
    {
        // Precalculated paramters
        HashSet<SimObject> ignore = new HashSet<SimObject>();
        ignore.add(MonteCarloBot.this);
        double radstep = Math.atan2(gridmap_meters_per_pixel, gridmap_range);
        double minDeg = -135;
        double maxDeg = 135;
        double maxRange = 29.9;
        double rad0 = Math.toRadians(minDeg);
        double rad1 = Math.toRadians(maxDeg);
        laser_t laser = new laser_t();
        laser.utime = TimeUtil.utime();

        // While control law has not finished OR timeout, try updating
        int timeout = (int)(10.0/FastDrive.DT);
        while (!test.conditionMet() && timeout > 0) {
            // LASER UPDATE
            double[][] T_truth = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                                                        drive.poseTruth.pos),
                                                 LinAlg.translate(0.3, 0, 0.25));
            double[][] T_odom = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseOdom.orientation,
                                                                       drive.poseOdom.pos),
                                                LinAlg.translate(0.3, 0, 0.25));

            double ranges[] = Sensors.laser(sw, ignore, T_truth, (int) ((rad1-rad0)/radstep), rad0, radstep, maxRange);

            double mean = 0;
            double stddev = 0.01;
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
            drive.update();

            // CHECK CLASSIFICATIONS
            // XXX

            laser.utime += FastDrive.DT*1000000;

            // TIME UPDATE
            timeout--;
        }
    }

    // === SimObject interface ===============================
    public double[][] getPose()
    {
        return null;    // XXX
    }

    public void setPose(double[][] T)
    {
        // XXX
    }

    public Shape getShape()
    {
        return null;    // XXX
    }

    static Model4 model4 = new Model4();
    public VisObject getVisObject()
    {
        return model4;
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
