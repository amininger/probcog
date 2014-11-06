package probcog.slam;

import java.awt.*;
import java.awt.event.*;
import java.awt.image.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;
import april.vis.*;

import magic2.lcmtypes.*; // XXX

public class MapViewer
{
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    TagMap map;

    private class LaserClickHandler extends VisEventAdapter
    {
        double[] clickXY = null;
        double[] endXY = null;

        boolean newLaser = false;
        double[] xyt = null;
        laser_t laser = null;

        boolean newTags = false;
        ArrayList<TagMap.TagXYT> seenTags = null;

        public boolean mousePressed(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            int mods = e.getModifiersEx();
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) != 0;
            boolean m1 = (mods & InputEvent.BUTTON1_DOWN_MASK) != 0;

            if (ctrl && m1) {
                clickXY = LinAlg.resize(ray.intersectPlaneXY(), 2);
                endXY = clickXY;
                drawHandler();
                return true;
            }

            return false;
        }

        public boolean mouseDragged(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            int mods = e.getModifiersEx();
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) != 0;
            boolean m1 = (mods & InputEvent.BUTTON1_DOWN_MASK) != 0;

            if (ctrl && m1) {
                endXY = LinAlg.resize(ray.intersectPlaneXY(), 2);
                drawHandler();
                return true;
            }

            return false;
        }

        public boolean mouseReleased(VisCanvas vc, VisLayer vl, VisCanvas.RenderInfo rinfo, GRay3D ray, MouseEvent e)
        {
            int mods = e.getModifiersEx();
            boolean ctrl = (mods & MouseEvent.CTRL_DOWN_MASK) != 0;
            boolean m1 = (mods & InputEvent.BUTTON1_DOWN_MASK) != 0;

            if (clickXY != null) {
                // Create an XYT and get a laser_t to render
                xyt = LinAlg.resize(clickXY, 3);
                xyt[2] = MathUtil.mod2pi(Math.atan2(endXY[1]-clickXY[1],
                                                    endXY[0]-clickXY[0]));
                laser = map.getLaser(xyt);
                newLaser = true;

                // Also, get a list of relative tag positions
                seenTags = map.getTags(xyt);
                newTags = true;

                clickXY = null;
                endXY = null;
                drawHandler();
                return true;
            }

            return false;
        }

        private void drawHandler()
        {
            VisWorld.Buffer vb = vw.getBuffer("debug-tagmap");
            vb.setDrawOrder(100);
            if (clickXY != null) {
                VisVertexData vvd = new VisVertexData();
                vvd.add(clickXY);
                vvd.add(endXY);
                vb.addBack(new VzLines(vvd,
                                       VzLines.LINES,
                                       new VzLines.Style(Color.yellow, 2)));
            }

            if (laser != null && newLaser) {
                ArrayList<double[]> points = new ArrayList<double[]>();
                for (int i = 0; i < laser.nranges; i++) {
                    double t = laser.rad0 + i*laser.radstep;
                    double r = laser.ranges[i];
                    if (r < 0)
                        continue;
                    double[] xy = new double[] {r*Math.cos(t),
                                                r*Math.sin(t)};
                    points.add(xy);
                }

                vb.addBack(new VisChain(LinAlg.xytToMatrix(xyt),
                                        new VzPoints(new VisVertexData(points),
                                                     new VzPoints.Style(Color.red, 2))));
                newLaser = false;
            }

            if (seenTags != null && newTags) {
                for (TagMap.TagXYT tag: seenTags) {
                    vb.addBack(new VisChain(LinAlg.xytToMatrix(LinAlg.xytMultiply(xyt, tag.xyt)),
                                            new VzAxes()));
                }

                newTags = false;
            }

            vb.swap();
        }
    }

    public MapViewer(String filename)
    {
        initGui();
        try {
            StructureReader fin = new BinaryStructureReader(new BufferedInputStream(new FileInputStream(filename)));
            map = TagMap.load(fin);
            render();
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }

    private void initGui()
    {
        JFrame jf = new JFrame("Map Viewer");
        jf.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        jf.setLayout(new BorderLayout());
        jf.setSize(1280, 720);

        vw = new VisWorld();
        vl = new VisLayer(vw);
        vc = new VisCanvas(vl);
        vl.addEventHandler(new LaserClickHandler());
        jf.add(vc, BorderLayout.CENTER);

        jf.setVisible(true);
    }

    private void render()
    {
        VisWorld.Buffer vb = vw.getBuffer("map");
        vb.setDrawOrder(-10);
        vb.addBack(new VisChain(LinAlg.translate(map.gm.x0, map.gm.y0),
                                LinAlg.scale(map.gm.metersPerPixel),
                                new VzImage(map.gm.makeBufferedImage())));
        vb.swap();

        vb = vw.getBuffer("tags");
        vb.setDrawOrder(10);
        for (int i = 0; i < map.tags.size(); i++) {
            TagMap.TagXYT tag = map.tags.get(i);
            String f = String.format("%d", tag.id);
            vb.addBack(new VisChain(LinAlg.xytToMatrix(tag.xyt),
                                    LinAlg.scale(0.1),
                                    new VzText(VzText.ANCHOR.CENTER,
                                               f)));
        }
        vb.swap();
    }

    static public void main(String[] args)
    {
        String filename = args[0];
        new MapViewer(filename);
    }
}
