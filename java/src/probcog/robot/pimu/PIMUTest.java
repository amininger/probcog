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

import lcm.lcm.*;

public class PIMUTest implements PIMU.PIMUListener, ParameterListener
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

    PIMUFilter filter = new PIMUFilter();

    public PIMUTest()
    {
        pg = new ParameterGUI();
        pg.addDoubleSlider("maxintperiod", "Acceleration integration period (s)", 0.5, 30, filter.maxintperiod);
        pg.addDoubleSlider("maxv", "Maximum vehicle velocity (m/s)", 1, 10, filter.maxv);
        pg.addButtons("reset", "Reset orientation");

        pg.addListener(this);

        jf = new JFrame("PIMUTest");
        jf.setLayout(new BorderLayout());
        jf.add(vc, BorderLayout.CENTER);
        jf.add(pg, BorderLayout.SOUTH);
        jf.setSize(600,400);
        jf.setVisible(true);

        vl.cameraManager.uiLookAt(new double[] {0, -0.14, 0.07},
                                  new double[] {0, 0, 0},
                                  new double[] {0, 0.54 , 0.837}, true);
        vw.getBuffer("grid").addFront(new VzGrid());
    }

    public void parameterChanged(ParameterGUI pg, String name)
    {
        filter.maxintperiod = pg.gd("maxintperiod");
        filter.maxv = pg.gd("maxv");

        if (name.equals("reset"))
            filter.setQuaternion(LinAlg.rollPitchYawToQuat(new double[3]));
    }

    public void pimuData(pimu_t pimu)
    {
        handle(pimu);
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            pimu_t pimu = new pimu_t(ins);
            handle(pimu);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }

    public void handle(pimu_t pimu)
    {
        filter.update(pimu);

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
                                                    VzLines.LINE_STRIP,
                                                    new VzLines.Style(Color.blue, 2))));

            vb.addBack(new VisChain(filter.getMatrix(),
                                    LinAlg.scale(.1, .1, .1),
                                    new VzLines(new VisVertexData(new double[] {0,0,0},
                                                                  acceldir),
                                                VzLines.LINE_STRIP,
                                                new VzLines.Style(Color.black, 2)),
                                    LinAlg.quatToMatrix(q)//,
                                    // vis2
                                    // new VzCylinder(0, Math.sin(filter.getUpUncertainty()),
                                    //                 0, 1,
                                    //                 new Color(128,128,128,128))
                           ));
        }

        vb.addBack(new VisChain(filter.getMatrix(),
                                    LinAlg.scale(.0001, .0001, .0001),
                                    new VzLines(new VisVertexData(new double[] {0,0,0},
                                                                   new double[] { pimu.mag[0],
                                                                                  pimu.mag[1],
                                                                                  pimu.mag[2] }),
                                                VzLines.LINE_STRIP,
                                                new VzLines.Style(Color.red, 4))));

        double rpy[] = LinAlg.quatToRollPitchYaw(filter.getQuaternion());

        vb.addBack(new VisPixCoords(VisPixCoords.ORIGIN.BOTTOM_RIGHT,
                                           new VzText(VzText.ANCHOR.BOTTOM_RIGHT,
                                                      String.format("<<right>>rpy deg = [ %10.3f %10.3f %10.3f ]", Math.toDegrees(rpy[0]), Math.toDegrees(rpy[1]), Math.toDegrees(rpy[2])))));

        vb.swap();
   }

    public static void main(String args[])
    {
        PIMUTest pt = new PIMUTest();

        if (args.length > 0) {
            System.out.println("Opening device at "+args[0]);

            GetOpt gopt = new GetOpt();
            gopt.addString('d', "device", args[0], "Device path");

            PIMU imu = new PIMU(gopt);
            imu.addListener(pt);
        } else {
            System.out.println("Listening on LCM for PIMU");
            LCM lcm = LCM.getSingleton();
//            lcm.subscribe("PIMU", pt);
        }

    }
}
