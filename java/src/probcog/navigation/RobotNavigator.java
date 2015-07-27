package probcog.navigation;

import java.awt.*;
import java.io.*;
import java.util.*;
import javax.swing.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.sim.*;
import april.tag.*;
import april.util.*;
import april.vis.*;

import probcog.commands.controls.*;
import probcog.commands.tests.*;
import probcog.lcmtypes.*;
import probcog.navigation.*;
import probcog.sim.*;
import probcog.util.*;

import magic2.lcmtypes.*;

/** Run on robot to do planning based on a map (possible supplied at
 *  runtime, otherwise, preconstructed) in control law space.
 **/
public class RobotNavigator implements LCMSubscriber
{
    private boolean DEBUG = true;
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    boolean executing = false;
    ExecutionThread exec = null;

    plan_request_t lastRequest = null;

    LCM lcm = LCM.getSingleton();
    String tagChannel;
    String planChannel;

    SimWorld world;
    GridMap gm; // Eventually populated by SLAM, for now, populated by sim?

    int commandID = 0;
    private class ExecutionThread extends Thread implements LCMSubscriber
    {
        control_law_status_t lastStatus = null;
        boolean statusMessageReceived = false;
        Object statusLock = new Object();
        ArrayList<Behavior> plan;

        public ExecutionThread(ArrayList<Behavior> plan_)
        {
            plan = plan;
            executing = true;
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                if (channel.equals("SOAR_COMMAND_STATUS_TX")) {
                    // Handle command statuses...see if we can move on
                    synchronized (statusLock) {
                        control_law_status_t status = new control_law_status_t(ins);
                        if (status.id == commandID) {
                            lastStatus = status;
                            statusMessageReceived = true;
                            statusLock.notifyAll();
                        }
                    }
                }
            } catch (IOException ex) {
                System.err.println("ERR: Could not handle message on channel - "+channel);
                ex.printStackTrace();
            }
        }

        // XXX TODO Execute the current plan. This means issuing the command
        // and then making sure we can respond appropriately as status messages
        // come in.
        public void run()
        {
            for (Behavior b: plan) {
                synchronized (statusLock) {
                    statusMessageReceived = false;
                    do {
                        issueCommand(b);
                        try {
                            statusLock.wait(100);
                        } catch (InterruptedException ex) {}
                    } while (!statusMessageReceived);

                    while (lastStatus.status.equals("EXECUTING")) {
                        try {
                            statusLock.wait();
                        } catch (InterruptedException ex) {}
                    }

                    System.out.printf("NFO: Execution %s on %d\n",
                                      lastStatus.status,
                                      lastStatus.id);
                    commandID++;
                }
            }
        }

        private void issueCommand(Behavior b)
        {
            // Issue the appropriate command
            control_law_t cl = null;
            if (b.law instanceof FollowWall) {
                cl = ((FollowWall)b.law).getLCM();
            } else if (b.law instanceof DriveTowardsTag) {
                cl = ((DriveTowardsTag)b.law).getLCM();
            } else if (b.law instanceof Turn) {
                cl = ((Turn)b.law).getLCM();
            } else if (b.law instanceof Orient) {
                cl = ((Orient)b.law).getLCM();
            } else {
                assert (false);
            }
            cl.utime = TimeUtil.utime();
            cl.id = commandID;

            condition_test_t ct = null;
            if (b.test instanceof ClassificationCounterTest) {
                ct = ((ClassificationCounterTest)b.test).getLCM();
            } else if (b.test instanceof NearTag) {
                ct = ((NearTag)b.test).getLCM();
            } else if (b.test instanceof RotationTest) {
                ct = ((RotationTest)b.test).getLCM();
            } else if (b.test instanceof Stabilized) {
                ct = ((Stabilized)b.test).getLCM();
            } else {
                assert (false);
            }
            cl.termination_condition = ct;

            // Publishing
            lcm.publish("SOAR_COMMAND", cl);
            lcm.publish("SOAR_COMMAND", cl);
            lcm.publish("SOAR_COMMAND", cl);
        }
    }

    public RobotNavigator(GetOpt gopt) throws IOException
    {
        if (gopt.wasSpecified("world")) {
            Config config = new Config();
            world = new SimWorld(gopt.getString("world"), config);
            world.setRunning(false);

            createGridMap();
        } else {
            System.err.println("ERR: Currently only support sim mode");
            assert (false);
        }

        if (DEBUG) {
            JFrame jf = new JFrame("Debug RobotNavigator");
            jf.setSize(1000, 800);
            jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
            jf.setLayout(new BorderLayout());

            vw = new VisWorld();
            vl = new VisLayer(vw);
            vc = new VisCanvas(vl);
            jf.add(vc, BorderLayout.CENTER);
            Simulator sim = new Simulator(vw, vl, new VisConsole(vw, vl, vc), world);

            jf.setVisible(true);
        }

        tagChannel = gopt.getString("tag-channel");
        lcm.subscribe(tagChannel, this);
        planChannel = gopt.getString("plan-channel");
        lcm.subscribe(planChannel, this);
    }

    private void createGridMap()
    {
        if (world.objects.size() < 1)
            return;

        // Set dimensions
        double max[] = {-Double.MAX_VALUE, - Double.MAX_VALUE,- Double.MAX_VALUE};
        double min[] = {Double.MAX_VALUE, Double.MAX_VALUE, Double.MAX_VALUE};
        for (SimObject so : world.objects) {
            double T[][] = so.getPose();
            april.sim.Shape s = so.getShape();
            if (s instanceof BoxShape) {
                BoxShape bs = (BoxShape) s;

                ArrayList<double[]> vertices = bs.getVertices();

                for (double vertex[] : vertices) {
                    double global_v[] = LinAlg.transform(T, vertex);

                    for (int l = 0; l < 3; l++) {
                        max[l] = Math.max(global_v[l],max[l]);
                        min[l] = Math.min(global_v[l],min[l]);
                    }
                }

            } else if (s instanceof SphereShape){
                SphereShape ss = (SphereShape) s;
                double r = ss.getRadius();
                for (int l = 0; l < 3; l++) {
                    max[l] = Math.max(T[l][3] + r, max[l]);
                    min[l] = Math.min(T[l][3] - r, min[l]);
                }

            } else {
                for (int l = 0; l < 3; l++) {
                    max[l] = Math.max(T[l][3],max[l]);
                    min[l] = Math.min(T[l][3],min[l]);
                }
                System.out.println("WRN: Unsupported shape type: "+s.getClass().getName());
            }
        }

        double MPP = 0.1;
        double[] down = new double[] {0, 0, -1};
        gm = GridMap.makeMeters(min[0], min[1], max[0]-min[0], max[1]-min[1], MPP, 255);

        // XXX There's probably a faster way to do this, but this was easy and it's
        // a one-time thing
        for (double y = min[1]; y < max[1]; y+=.5*MPP) {
            for (double x = min[0]; x < max[0]; x+=.5*MPP) {
                for (SimObject obj: world.objects) {
                    if (!(obj instanceof SimBox))
                        continue;
                    if (Collisions.collisionDistance(new double[] {x, y, 100}, down, obj.getShape(), obj.getPose()) < Double.MAX_VALUE) {
                        gm.setValue(x, y, (byte)0);
                    }
                }
            }
        }
    }

    /** Given a goal in global XYT, compute a plan to execute in control law
     *  space.
     *
     *  Note that the robot will localize itself based on tags, teleporting
     *  around the sim world as needed.
     **/
    public ArrayList<Behavior> computePlan(double[] goalXYT)
    {
        MonteCarloPlanner mcp = new MonteCarloPlanner(world, gm, null);

        // For now, base starting position on wherever the robot was
        // last placed in the world and the closest tag to our target
        // goal. Maybe someday we will have better localization and the ability
        // to GOTO XY and be a little smarter about final goals/termination
        // conditions.
        SimRobot robot = getRobot();
        assert (robot != null);
        SimAprilTag tag = getClosestTag(goalXYT);
        assert (tag != null);

        ArrayList<double[]> starts = new ArrayList<double[]>();
        starts.add(LinAlg.matrixToXYT(robot.getPose()));

        return mcp.plan(starts, goalXYT, tag);
    }

    // === LCM ===
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            // Handle LCM messages as appropriate. For example, we will try to
            // localize off tag detections relative to actual positions in the
            // real world.
            if (channel.equals(tagChannel)) {
                tag_detection_list_t tdl = new tag_detection_list_t(ins);
                handleTags(tdl);
            } else if (channel.equals(planChannel)) {
                plan_request_t pr = new plan_request_t(ins);
                handleRequest(pr);
            }
        } catch (IOException ex) {
            System.err.println("ERR: Could not handle message on channel - "+channel);
            ex.printStackTrace();
        }
    }

    synchronized private void handleTags(tag_detection_list_t tdl)
    {
        // Correct robot pose based on tags. In the presence of multiple
        // tags from the sim world, use the average position.
        // XXX We need to eliminate tags that might also be robots! Currently,
        // we assume that the tags will provide (noisy) estimates of XYT.

        double tagSize_m = Util.getConfig().requireDouble("tag_detection.tag.size_m");

        double[] meanXYT = new double[3];
        for (int i = 0; i < tdl.ndetections; i++) {
            tag_detection_t td = tdl.detections[i];

            // XXX Calibration of detection. Should it happen on this end,
            // or on the detection side?
            TagDetection tag = new TagDetection();
            tag.id = td.id;
            tag.hammingDistance = td.hamming_dist;
            tag.homography = new double[3][];
            for (int n = 0; n < 3; n++) {
                tag.homography[n] = new double[3];
                for (int m = 0; m < 3; m++) {
                    tag.homography[n][m] = (double)(td.H[n][m]);
                }
            }
            tag.cxy = new double[] {td.cxy[0], td.cxy[1]};
            tag.p = new double[][] {{td.pxy[0][0], td.pxy[0][1]},
                                    {td.pxy[1][0], td.pxy[1][1]},
                                    {td.pxy[2][0], td.pxy[2][1]},
                                    {td.pxy[3][0], td.pxy[3][1]}};

            // Compute the tag position relative to the robot
            // for a fixed, upwards facing camera.
            double[][] T2B = TagUtil.getTagToPose(tag, tagSize_m);

            // Determine the actual tag position
            SimAprilTag simTag = getTagByID(tag.id);
            assert (simTag != null);
            double[][] T2W = simTag.getPose();

            // Use this to extract the global position of the robot
            // XXX Are we getting rotations right? Translation is pretty
            // trivial.
            double[][] B2W = LinAlg.matrixAB(T2W, LinAlg.inverse(T2B));
            double[] xyt = LinAlg.matrixToXYT(B2W);
            LinAlg.plusEquals(meanXYT, LinAlg.scale(xyt, 1.0/tdl.ndetections));
        }

        SimRobot robot = getRobot();
        assert (robot != null);
        double[][] B2W = LinAlg.xytToMatrix(meanXYT);
        B2W[2][3] = robot.getPose()[2][3];

        // XXX Filtering...also, take into account robot's reported odom
        robot.setPose(B2W);

        // XXX TODO
        // We need to...
        // 1) Localize off of tags in the sim world (might want to visualize
        // for this, after all?)
        //
        // 2) Make an interface, maybe via LCM, for querying the system for
        // plans
        //
        // 3) Have some other thread running in the background that actually
        // handles the execution of plans, resulting in a robot that DOES
        // something.
        //
        // XXX THE SIM SHOULD NOT DO ANYTHING, RECALL! JUST THE REAL ROBOT
    }

    synchronized private void handleRequest(plan_request_t pr)
    {
        if (lastRequest != null && lastRequest.id == pr.id)
            return;
        lastRequest = pr;

        // Compute and save plan for execution
        ArrayList<Behavior> plan = computePlan(pr.goalXYT);
        if (plan.size() < 0) {
            System.err.println("ERR: Zero length plan returned");
            return;
        }

        executing = false;
        try {
            if (exec != null)
                exec.join();
        } catch (InterruptedException ex) {}
        exec = new ExecutionThread(plan);
        exec.start();
    }

    // === UTILITY ===
    private SimRobot getRobot()
    {
        assert (world != null);
        for (SimObject so: world.objects) {
            if (!(so instanceof SimRobot))
                continue;
            return (SimRobot)so;
        }

        return null;
    }

    private SimAprilTag getTagByID(double id)
    {
        for (SimObject so: world.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            SimAprilTag tag = (SimAprilTag)so;
            if (tag.getID() == id)
                return tag;
        }

        return null;
    }

    private SimAprilTag getClosestTag(double[] xy)
    {
        SimAprilTag tag = null;
        double dist = Double.MAX_VALUE;
        for (SimObject so: world.objects) {
            if (!(so instanceof SimAprilTag))
                continue;
            double d = LinAlg.distance(xy, LinAlg.matrixToXYT(so.getPose()), 2);
            if (d < dist) {
                dist = d;
                tag = (SimAprilTag)so;
            }
        }

        return tag;
    }

    // =========================================================================
    static public void main(String[] args)
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help screen");
        gopt.addString('w', "world", null, "(Someday Optional) sim world file");
        gopt.addString('\0', "tag-channel", "TAG_DETECTIONS", "Tag detection channel");
        gopt.addString('\0', "plan-channel", "PLAN_REQUEST", "Plan request channel");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            System.err.printf("Usage: %s [options]\n", args[0]);
            gopt.doHelp();
            System.exit(1);
        }

        try {
            new RobotNavigator(gopt);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
