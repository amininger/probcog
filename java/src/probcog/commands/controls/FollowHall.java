package probcog.commands.controls;

// XXX Test
import java.awt.*;
import javax.swing.*;
import april.vis.*;
import april.jmat.geom.*;
// ----------------

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.lcmtypes.*;
import april.util.*;

import probcog.commands.*;
import probcog.lcmtypes.*;

public class FollowHall implements ControlLaw, LCMSubscriber
{
    PeriodicTasks tasks = new PeriodicTasks(1);
    ExpiringMessageCache<pose_t> poseCache = new ExpiringMessageCache<pose_t>(0.2);
    ExpiringMessageCache<laser_t> laserCache = new ExpiringMessageCache<laser_t>(0.2);

    /** Strictly for use in parameter checking */
    public FollowHall()
    {
        // Temporary
        LCM.getSingleton().subscribe("LASER", this);
    }

    public FollowHall(Map<String, TypedValue> parameters)
    {
        // Parameters

        // Add task
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.err.println("WRN: Error receving message from channel " + channel + ": "+ex);
        }
    }

    synchronized void messageReceivedEx(LCM lcm, String channel,
            LCMDataInputStream ins) throws IOException
    {
        if ("LASER".equals(channel)) {
            laser_t laser = new laser_t(ins);
            laserCache.put(laser, laser.utime);
        } else if ("POSE".equals(channel)) {
            pose_t pose = new pose_t(ins);
            poseCache.put(pose, pose.utime);
        }
    }

    /** Start/stop the execution of the control law.
     *
     *  @param run  True causes the control law to begin execution, false stops it
     **/
    public void setRunning(boolean run)
    {
        tasks.setRunning(run);
    }

    /** Get the name of this control law. Mostly useful for debugging purposes.
     *
     *  @return The name of the control law
     **/
    public String getName()
    {
        return "FOLLOW_HEADING";
    }

    /** Get the parameters that can be set for this control law.
     *
     *  @return An iterable collection of all possible parameters
     **/
    public Collection<TypedParameter> getParameters()
    {
        ArrayList<TypedParameter> params = new ArrayList<TypedParameter>();

        return params;
    }

    // ======================================================================
    // === TEST                                                           ===
    // ======================================================================

    class TestThread extends Thread
    {
        int fps = 60;
        VisWorld vw;

        public TestThread(VisWorld vw)
        {
            this.vw = vw;
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/fps);
                laser_t laser = laserCache.get();
                if (laser == null)
                    continue;

                process(laser);
            }
        }

        private void process(laser_t laser)
        {
            ArrayList<double[]> points = new ArrayList<double[]>();
            for (int i = 0; i < laser.nranges; i++) {
                double t = laser.rad0 + i*laser.radstep;
                double r = laser.ranges[i];
                points.add(new double[] {r*Math.cos(t), r*Math.sin(t)});
            }

            // Render points
            VisWorld.Buffer vb = vw.getBuffer("laser");
            vb.addBack(new VzPoints(new VisVertexData(points),
                                    new VzPoints.Style(Color.yellow, 2)));
            vb.swap();

            // Locate the direction closest to being in front of us (without
            // being behind us) that is free.
            // (Later, we'll add some slight push back by walls to try to recenter)
            double MIN_RANGE = 3.0; // [m]
            double THETA_SWEEP = Math.atan2(0.5, MIN_RANGE);
            double MIN_THETA = -Math.PI/2 - THETA_SWEEP/2;
            double MAX_THETA = Math.PI/2 + THETA_SWEEP/2;
            int SAMPLES = (int)Math.ceil(THETA_SWEEP/laser.radstep);
            LinkedList<Double> slidingRanges = new LinkedList<Double>();
            int unsafe = 0;
            double bestTheta = Double.MAX_VALUE;
            for (int i = 0; i < laser.nranges; i++) {
                double t = laser.rad0 + i*laser.radstep;
                if (t < MIN_THETA || t > MAX_THETA)
                    continue;
                if (laser.ranges[i] < MIN_RANGE)
                    unsafe++;
                slidingRanges.addLast((double)laser.ranges[i]);
                if (slidingRanges.size() > SAMPLES) {
                    double r = slidingRanges.removeFirst();
                    if (r < MIN_RANGE)
                        unsafe--;
                }
                if (slidingRanges.size() != SAMPLES)
                    continue;

                double middleTheta = laser.rad0 + (i-SAMPLES/2)*laser.radstep;
                if (unsafe == 0 && Math.abs(middleTheta) < Math.abs(bestTheta))
                    bestTheta = middleTheta;
            }

            double radius = MIN_RANGE;
            if (bestTheta == Double.MAX_VALUE) {
                radius = 0;
                bestTheta = 0;
            }
            vb = vw.getBuffer("direction");
            VisVertexData vvd = new VisVertexData(new double[2], new double[] {radius*Math.cos(bestTheta), radius*Math.sin(bestTheta)});
            vb.addBack(new VzLines(vvd, VzLines.LINES, new VzLines.Style(Color.red, 2)));
            vb.swap();
        }
    }

    public void launchTest(VisWorld vw)
    {
        (new TestThread(vw)).start();
    }

    // Test functionality
    public static void main(String[] args)
    {
        JFrame jf = new JFrame("Hall following test");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 800);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        VzGrid.addGrid(vw);
        jf.add(vc, BorderLayout.CENTER);

        jf.setVisible(true);

        (new FollowHall()).launchTest(vw);
    }
}
