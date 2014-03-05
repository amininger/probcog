package probcog.sensor;

import java.awt.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import april.util.*;
import april.vis.*;

import probcog.util.*;

/** Visualize raw sensor data */
public class SensorTest
{
    Sensor s;

    public SensorTest(Sensor s)
    {
        JFrame jf = new JFrame("Sensor Test");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(800, 600);

        VisWorld vw = new VisWorld();
        VisLayer vl = new VisLayer(vw);
        VisCanvas vc = new VisCanvas(vl);
        jf.add(vc, BorderLayout.CENTER);

        this.s = s;

        (new RenderThread(vw)).start();

        jf.setVisible(true);
    }

    class RenderThread extends Thread
    {
        int fps = 30;
        VisWorld vw;

        public RenderThread(VisWorld vw)
        {
            this.vw = vw;
        }

        public void run()
        {
            vw.getBuffer("axes").addBack(new VzAxes());
            vw.getBuffer("axes").swap();
            while (true) {
                if (s.stashFrame()) {
                    ArrayList<double[]> xyzrgb = s.getAllXYZRGB();
                    VisVertexData vvd = new VisVertexData(xyzrgb);
                    VisColorData vcd = new VisColorData();
                    for (double[] d: xyzrgb) {
                        vcd.add((int)d[3]);
                    }

                    VisWorld.Buffer vb = vw.getBuffer("points");
                    vb.addBack(new VzPoints(vvd, new VzPoints.Style(vcd, 2)));
                    vb.swap();
                }

                TimeUtil.sleep(1000/fps);
            }
        }
    }

    static public void main(String[] args)
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h', "help", false, "Show this help screen");
        // Later, could add support for appropriate sensor. For now,
        // always a real kinect.

        // Meh
        if (!opts.parse(args) || opts.getBoolean("help")) {
            System.err.println(opts.getReason());
            opts.doHelp();
            System.exit(1);
        }

        try {
            Sensor s = new KinectSensor();
            new SensorTest(s);
        } catch (IOException ioex) {
            ioex.printStackTrace();
        }
    }


}
