package probcog.navigation;

import java.awt.*;
import java.io.*;
import java.util.*;
import javax.swing.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

public class GraphDataConverter
{
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;
    ParameterGUI pg;

    public ArrayList<TrialData> data;

    private class InputListener implements ParameterListener
    {
        public void parameterChanged(ParameterGUI pg, String name)
        {
            //if (name.equals("graph")) {
                generateGraph();
            //}
        }
    }

    private class PlanData
    {
        public int startTag;
        public int endTag;
        public double prob;
        public double dist;
    }

    private class TrialData
    {
        public double fullExecutionTime;
        public double timePerTree;
        public double desiredTimePerTree;

        public ArrayList<PlanData> planDatas = new ArrayList<PlanData>();
    }

    public GraphDataConverter(GetOpt opts) throws IOException
    {
        // Load data
        String filename = opts.getExtraArgs().get(0);
        TextStructureReader fin = null;
        fin = new TextStructureReader(new BufferedReader(new FileReader(filename)));

        double lambda = fin.readDouble();
        String worldname = fin.readString();
        int numSamples = fin.readInt();

        // Now, read the actual data in
        int minTag = Integer.MAX_VALUE;
        int maxTag = Integer.MIN_VALUE;
        ArrayList<TrialData> trialDatas = new ArrayList<TrialData>();
        for (int i = 0; i < numSamples; i++) {
            TrialData trialData = new TrialData();
            trialData.fullExecutionTime = fin.readDouble();
            trialData.timePerTree = fin.readDouble();
            trialData.desiredTimePerTree = fin.readDouble();
            int numPlans = fin.readInt();
            for (int j = 0; j < numPlans; j++) {
                PlanData planData = new PlanData();
                fin.blockBegin();
                planData.startTag = fin.readInt();
                planData.endTag = fin.readInt();
                planData.prob = fin.readDouble();
                planData.dist = fin.readDouble();
                fin.blockEnd();

                minTag = Math.min(minTag, planData.startTag);
                minTag = Math.min(minTag, planData.endTag);
                maxTag = Math.max(maxTag, planData.startTag);
                maxTag = Math.max(maxTag, planData.endTag);

                trialData.planDatas.add(planData);
            }
            trialDatas.add(trialData);
        }

        data = trialDatas;

        // Spin up gui
        JFrame jf = new JFrame("Graph Data Converter");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        pg = new ParameterGUI();
        pg.addIntSlider("start", "Start Tag ID", minTag, maxTag, minTag);
        pg.addIntSlider("end", "End Tag ID", minTag, maxTag, maxTag);
        pg.addButtons("graph", "Generate Graph");
        pg.addListener(new InputListener());
        jf.add(pg, BorderLayout.SOUTH);

        jf.setVisible(true);
    }

    private void generateGraph()
    {
        int start = pg.gi("start");
        int end = pg.gi("end");

        if (start == end) {
            System.err.println("ERR: Cannot plan from a location to itself");
            return;
        }

        double maxTime = data.get(0).timePerTree;
        double maxDist = 0;
        ArrayList<double[]> prob = new ArrayList<double[]>();
        ArrayList<double[]> dist = new ArrayList<double[]>();
        for (TrialData trialData: data) {
            double time = trialData.timePerTree;
            PlanData planData = null;
            for (PlanData pd: trialData.planDatas) {
                if (pd.startTag == start && pd.endTag == end) {
                    planData = pd;
                    break;
                }
            }

            assert (planData != null);
            prob.add(new double[] {time/maxTime, planData.prob});
            dist.add(new double[] {time/maxTime, planData.dist});
            maxDist = Math.max(maxDist, planData.dist);
        }

        for (double[] d: dist) {
            d[1] /= maxDist;
        }

        VisWorld.Buffer vb = vw.getBuffer("axes");
        vb.setDrawOrder(-10);
        vb.addBack(new VisChain(LinAlg.scale(0.5),
                                LinAlg.translate(1, 1),
                                new VzRectangle(new VzLines.Style(Color.white, 1))));
        vb.swap();

        vb = vw.getBuffer("prob-data");
        vb.setDrawOrder(0);
        vb.addBack(new VzLines(new VisVertexData(prob),
                               VzLines.LINE_STRIP,
                               new VzLines.Style(Color.red, 2)));
        vb.addBack(new VzPoints(new VisVertexData(prob),
                                new VzPoints.Style(Color.yellow, 2)));
        vb.swap();

        vb = vw.getBuffer("dist-data");
        vb.setDrawOrder(5);
        vb.addBack(new VzLines(new VisVertexData(dist),
                               VzLines.LINE_STRIP,
                               new VzLines.Style(Color.blue, 2)));
        vb.addBack(new VzPoints(new VisVertexData(dist),
                                new VzPoints.Style(Color.cyan, 2)));
        vb.swap();
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");

        if (!opts.parse(args) || opts.getBoolean("help")) {
            System.out.println("Usage: java GraphDataConverter [options] <*.graph_data>");
            opts.doHelp();
            System.exit(1);
        }

        try {
            new GraphDataConverter(opts);
        } catch (IOException ioex) {
            System.err.println("ERR: Could not parse file - "+ioex);
            ioex.printStackTrace();
        }
    }
}
