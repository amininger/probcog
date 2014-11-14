package probcog.gui;

import java.awt.*;
import java.awt.image.*;
import javax.swing.*;
import java.util.*;
import java.io.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

/** Used to process our data files */
public class RenderData
{
    int numTrials;
    HashMap<TrajectoryType, TrajectoryData> tmap = new HashMap<TrajectoryType, TrajectoryData>();

    private enum TrajectoryType
    {
        PERFECT,
        WAVEFRONT,
        MONTE_CARLO
    }

    private class GoalStats
    {
        public double mean = 0;
        public double mean2 = 0;
        public double var = 0;
        public double min = Double.POSITIVE_INFINITY;
        public double max = Double.NEGATIVE_INFINITY;

        public String toString()
        {
            return String.format("mean: %f\nmean2: %f\nvar: %s\nmin: %f\nmax: %f\n",
                                 mean,
                                 mean2,
                                 var,
                                 min,
                                 max);
        }
    }

    private class TrajectoryData
    {
        public double[] range = new double[4]; // minx, maxx, miny, maxy
        public ArrayList<double[]> goals;
        public ArrayList<ArrayList<double[]> > trajectories;

        public TrajectoryData()
        {
            range[0] = range[2] = Double.POSITIVE_INFINITY;
            range[1] = range[3] = Double.NEGATIVE_INFINITY;
            goals = new ArrayList<double[]>();
            trajectories = new ArrayList<ArrayList<double[]> >();
        }

        public void createTrajectory(double[] goal)
        {
            goals.add(goal);
            trajectories.add(new ArrayList<double[]>());
        }

        public void addPoint(double[] xy)
        {
            assert (trajectories.size() > 0);
            trajectories.get(trajectories.size()-1).add(xy);
            range[0] = Math.min(xy[0], range[0]);
            range[1] = Math.max(xy[0], range[1]);
            range[2] = Math.min(xy[1], range[2]);
            range[3] = Math.max(xy[1], range[3]);
        }

        public GoalStats goalStats()
        {
            GoalStats gs = new GoalStats();
            if (goals.size() < 1)
                return gs;

            for (int i = 0; i < goals.size(); i++) {
                double[] goal = goals.get(i);
                ArrayList<double[]> t = trajectories.get(i);
                double dist = LinAlg.distance(goal, t.get(t.size()-1), 2);
                gs.mean += dist;
                gs.mean2 += dist*dist;
                gs.max = Math.max(gs.max, dist);
                gs.min = Math.min(gs.min, dist);
            }
            gs.mean /= goals.size();
            gs.mean2 /= goals.size();
            gs.var = gs.mean2 - gs.mean*gs.mean;

            return gs;
        }
    }

    public RenderData(GetOpt opts)
    {
        // Option 1: We are rending a trajectory
        if (opts.getString("trajectory") != null) {
            renderTrajectory(opts);
        }
    }

    private void renderTrajectory(GetOpt opts)
    {
        TextStructureReader fin;
        TrajectoryType type;
        try {
            fin = new TextStructureReader(new BufferedReader(new FileReader(opts.getString("trajectory"))));
            String tt = opts.getString("trajectory-type");
            assert (tt != null);
            if (tt.equals("wf"))
                type = TrajectoryType.WAVEFRONT;
            else if (tt.equals("mc"))
                type = TrajectoryType.MONTE_CARLO;
            else
                type = TrajectoryType.PERFECT;

            extractTrajectoryData(fin);

            ArrayList<TrajectoryType> types = new ArrayList<TrajectoryType>();
            //types.add(TrajectoryType.WAVEFRONT);
            //types.add(TrajectoryType.PERFECT);
            types.add(TrajectoryType.MONTE_CARLO);
            for (TrajectoryType t: types) {
                TrajectoryData data = tmap.get(t);
                // Render heatmap
                renderTrajectoryData(data);

                // Compute goal stats
                GoalStats gs = data.goalStats();
                System.out.println(t.name());
                System.out.println(gs);
            }


        } catch (IOException ex) {
            ex.printStackTrace();
            return;
        }
    }

    private void extractTrajectoryData(TextStructureReader fin)
        throws IOException
    {
        numTrials = fin.readInt();
        System.out.println("Parsing "+numTrials+" trials");

        ArrayList<TrajectoryType> types = new ArrayList<TrajectoryType>();
        //types.add(TrajectoryType.WAVEFRONT);
        //types.add(TrajectoryType.PERFECT);
        types.add(TrajectoryType.MONTE_CARLO);
        // Outer loop: handle algorithms
        for (TrajectoryType type: types) {
            TrajectoryData data = new TrajectoryData();
            for (int i = 0; i < numTrials; i++) {
                System.out.println("\t"+i);
                double[] goal = fin.readDoubles();
                data.createTrajectory(goal);
                int numPoints = fin.readInt();
                for (int j = 0; j < numPoints; j++) {
                    data.addPoint(fin.readDoubles());
                }
            }
            tmap.put(type, data);
        }
    }

    private void renderTrajectoryData(TrajectoryData data)
    {
        JFrame jf = new JFrame("Trajectory Heatmap");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        double mpp = 0.5;
        int width = (int)(Math.ceil((data.range[1]-data.range[0])/mpp))+1;
        int height = (int)(Math.ceil((data.range[3]-data.range[2])/mpp))+1;

        BufferedImage im = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        int[] buf = ((DataBufferInt)(im.getRaster().getDataBuffer())).getData();

        // Stats for rendering
        int maxHits = 0;
        int lastX, lastY;
        lastX = lastY = 0;

        // Ignores single point trajectories.
        for (ArrayList<double[]> t: data.trajectories) {
            boolean set = false;
            for (double[] xy: t) {
                int ix = (int)(Math.floor((xy[0]-data.range[0])/mpp));
                int iy = (int)(Math.floor((xy[1]-data.range[2])/mpp));

                // Trace a line between the points
                if (set) {
                    int dx = ix - lastX;
                    int dy = iy - lastY;
                    int dr = (int)(Math.floor(Math.sqrt(dx*dx + dy*dy)));
                    for (int i = 0; i < dr; i++) {
                        double p = (double)i/dr;
                        int x = Math.min((int)(Math.round(lastX + dx*p)), width-1);
                        int y = Math.min((int)(Math.round(lastY + dy*p)), height-1);
                        int val = buf[y*width + x]++;
                        maxHits = Math.max(maxHits, val);
                    }
                }

                lastX = ix;
                lastY = iy;
                set = true;
            }
        }
        int[] map = {
            0x000000,
            0x00ffff,
            0x00f6ff,
            0x00eeff,
            0x00e6ff,
            0x00deff,
            0x00d5ff,
            0x00cdff,
            0x00c5ff,
            0x00bdff,
            0x00b4ff,
            0x00acff,
            0x00a4ff,
            0x009cff,
            0x0094ff,
            0x008bff,
            0x0083ff,
            0x007bff,
            0x0073ff,
            0x006aff,
            0x0062ff,
            0x005aff,
            0x0052ff,
            0x004aff,
            0x0041ff,
            0x0039ff,
            0x0031ff,
            0x0029ff,
            0x0020ff,
            0x0018ff,
            0x0010ff,
            0x0008ff,
            0x0000ff};

        ColorMapper cm = new ColorMapper(map, 0, maxHits);
        cm = cm.swapRedBlue();
        for (int i = 0; i < buf.length; i++) {
            buf[i] = (cm.mapColor(buf[i])).getRGB();
        }

        // XXX Texture to handle blurring
        vw.getBuffer("image").addBack(new VisChain(LinAlg.scale(mpp),
                                                   new VzImage(new VisTexture(im,
                                                                              VisTexture.NO_MIN_FILTER |
                                                                              VisTexture.NO_MAG_FILTER))));

        vw.getBuffer("image").swap();

        jf.setVisible(true);
    }

    public static void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        opts.addString('w', "world", null, "Optional world data for rendering sim world");
        opts.addString('t', "trajectory", null, "Trajectory data");
        opts.addString('\0', "trajectory-type", null, "[perfect, wf, mc]");

        if (!opts.parse(args)) {
            System.err.println("Option error: "+opts.getReason());
            System.exit(-1);
        }

        if (opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(1);
        }

        new RenderData(opts);
    }
}
