package probcog.sim;

import java.awt.Color;
import java.io.*;
import java.util.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.vis.*;
import april.util.*;
import april.lcmtypes.*;

import probcog.vis.*;
import probcog.lcmtypes.*;
import probcog.classify.*;
import probcog.commands.*;
import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.util.*;

public class MonteCarloBot implements SimObject
{
    Random r = new Random();

    int ROBOT_ID = 6;
    SimWorld sw;

    Object runLock = new Object();
    boolean running = false;

    FastDrive drive = null;
    ArrayList<double[]> trajectoryTruth = new ArrayList<double[]>();
    ArrayList<double[]> trajectoryOdom = new ArrayList<double[]>();
    VisColorData vcd = new VisColorData();

    TagClassifier tc;
    FollowWall law;
    ClassificationCounterTest test;

    int simSinceReset = 0;
    boolean success = false;

    public MonteCarloBot(SimWorld sw)
    {
        this.sw = sw;
        try {
            this.tc = new TagClassifier(false);
        } catch (IOException ioex) {
            ioex.printStackTrace();
        }

    }

    // Track which tags we saw and, for each class for said tag, what that class
    // count for that tag was, aka, home many "doors" did we see before this "door"?
    private HashMap<String, Integer> classCount = new HashMap<String, Integer>();
    public HashSet<TagRecord> tagRecords = new HashSet<TagRecord>();
    static public class TagRecord
    {
        public int id;
        public int count;
        public String tagClass;

        public TagRecord(int id, int count, String tagClass)
        {
            this.id = id;
            this.count = count;
            this.tagClass = tagClass;
        }

        public int hashCode()
        {
            return new Integer(id).hashCode();
        }

        public boolean equals(Object o)
        {
            if (o == null)
                return false;
            if (!(o instanceof TagRecord))
                return false;
            TagRecord rec = (TagRecord)o;
            return id == rec.id;
        }
    }

    // === Random sampling interface =========================
    private void resetTrajectories()
    {
        simSinceReset = 0;
        vcd = new VisColorData();
        trajectoryTruth.clear();
        trajectoryOdom.clear();
        classCount.clear();
    }

    public void init(FollowWall law, ClassificationCounterTest test)
    {
        init(law, test, null);
    }

    public void init(FollowWall law, ClassificationCounterTest test, double[] xyt)
    {
        success = false;
        this.law = law;
        this.test = test;

        if (xyt != null) {
            drive = new FastDrive(sw, this, xyt);
            drive.centerOfRotation = new double[] { 0.13, 0, 0 };

            resetTrajectories();
            trajectoryTruth.add(drive.poseTruth.pos);
            trajectoryOdom.add(drive.poseOdom.pos);
        }
    }

    // Simulate all steps, keeping track of trajectory, etc.
    public void simulate()
    {
        simulate(Util.getConfig().requireDouble("monte_carlo.default_forward_search_time"));
    }

    public void simulate(double seconds)
    {
        simSinceReset++;

        // Precalculated paramters
        HashSet<SimObject> ignore = new HashSet<SimObject>();
        ignore.add(this);
        double radstep = Math.toRadians(Util.getConfig().requireDouble("monte_carlo.step_degrees"));
        double minDeg = Util.getConfig().requireDouble("monte_carlo.min_degrees");
        double maxDeg = Util.getConfig().requireDouble("monte_carlo.max_degrees");
        double maxRange = Util.getConfig().requireDouble("monte_carlo.max_range");
        double rad0 = Math.toRadians(minDeg);
        double rad1 = Math.toRadians(maxDeg);
        laser_t laser = new laser_t();
        laser.utime = TimeUtil.utime();

        // While control law has not finished OR timeout, try updating
        int timeout = (int)(seconds/FastDrive.DT);
        Tic tic = new Tic();
        double time = 0;
        while ((test == null || !test.conditionMet()) && timeout > 0) {
            tic.tic();
            // LASER UPDATE
            double[][] T_truth = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                                                        drive.poseTruth.pos),
                                                 LinAlg.translate(0.3, 0, 0.25));
            double[][] T_odom = LinAlg.matrixAB(LinAlg.quatPosToMatrix(drive.poseOdom.orientation,
                                                                       drive.poseOdom.pos),
                                                LinAlg.translate(0.3, 0, 0.25));

            // XXX This is where most of the low hanging fruit lies
            double ranges[] = Sensors.laser(sw, ignore, T_truth, (int) ((rad1-rad0)/radstep), rad0, radstep, maxRange);

            double mean = 0;
            double stddev = 0.01;   // Laser noise
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
            laser.utime += FastDrive.DT*1000000;

            // CHECK CLASSIFICATIONS
            double classificationRange = 2.0;  // Config
            for (SimObject so: sw.objects) {
                if (!(so instanceof SimAprilTag))
                    continue;
                double[] xyzrpy = LinAlg.matrixToXyzrpy(so.getPose());
                double d = LinAlg.distance(drive.poseTruth.pos, xyzrpy, 2);
                if (d > classificationRange)
                    continue;
                SimAprilTag tag = (SimAprilTag)so;
                double[] relXyzrpy = relativePose(getPose(), xyzrpy);
                ArrayList<classification_t> classies = tc.classifyTag(tag.getID(), relXyzrpy);

                // If the test object exists, add classification samples.
                // Otherwise, store relevant information about the tag. Only use
                // the FIRST tag class
                if (test == null) {
                    Set<String> tagClasses = tc.getClasses(tag.getID());
                    String c = tagClasses.iterator().next(); // Only ever have the one
                    if (!classCount.containsKey(c))
                        classCount.put(c, 0);

                    TagRecord rec = new TagRecord(tag.getID(), classCount.get(c)+1, c);
                    if (!tagRecords.contains(rec)) {
                        tagRecords.add(rec);
                        classCount.put(c, classCount.get(c)+1);
                    }
                } else {
                    for (classification_t classy: classies)
                        test.addSample(classy);
                }
            }



            // Visualization etc.
            trajectoryTruth.add(drive.poseTruth.pos);
            trajectoryOdom.add(drive.poseOdom.pos);
            if (simSinceReset%2 == 1)
                vcd.add(0xffff0000);  // BGR
            else
                vcd.add(0xffff00ff);  // BGR

            // TIME UPDATE
            timeout--;
            time += tic.toc();
            //System.out.printf("\t%f [s]\n", tic.toc());
        }
        success = timeout > 0;
        //System.out.printf("%f [s]\n", time);
    }

    // Convenience function for classification_t construction
    private double[] relativePose(double[][] A, double[] xyzrpy)
    {
        double[] xyzrpy_A = LinAlg.matrixToXyzrpy(A);
        double[] p = LinAlg.resize(xyzrpy, 3);
        p = LinAlg.transform(LinAlg.inverse(A), p);
        p = LinAlg.resize(p, 6);

        // Relative yaw difference.
        p[5] = MathUtil.mod2pi(xyzrpy[5] - xyzrpy_A[5]);

        return p;
    }

    public boolean success()
    {
        return success;
    }

    public double getTrajectoryLength()
    {
        if (trajectoryTruth.size() < 2)
            return 0;

        double length = 0;
        double[] prev = trajectoryTruth.get(0);
        for (int i = 1; i < trajectoryTruth.size(); i++) {
            double[] curr = trajectoryTruth.get(i);
            length += LinAlg.distance(curr, prev);
            prev = curr;
        }

        return length;
    }

    // === SimObject interface ===============================
    public double[][] getPose()
    {
        return LinAlg.quatPosToMatrix(drive.poseTruth.orientation,
                                      drive.poseTruth.pos);
    }

    public void setPose(double[][] T)
    {
        if (drive == null) {
            drive = new FastDrive(sw, this, LinAlg.matrixToXYT(T));
            drive.centerOfRotation = new double[] { 0.13, 0, 0 };
        }
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
                                               new VzLines.Style(vcd, 2)),
                                   //new VzLines(new VisVertexData(trajectoryOdom),
                                   //            VzLines.LINE_STRIP,
                                   //            new VzLines.Style(Color.red, 2)),
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
