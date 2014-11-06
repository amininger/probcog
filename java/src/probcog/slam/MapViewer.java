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
                // XXX Call laser render here
                clickXY = null;
                endXY = null;
                drawHandler();
                return true;
            }

            return false;
        }

        private void drawHandler()
        {
            VisWorld.Buffer vb = vw.getBuffer("laser");
            if (clickXY != null) {
                VisVertexData vvd = new VisVertexData();
                vvd.add(clickXY);
                vvd.add(endXY);
                vb.addBack(new VzLines(vvd,
                                       VzLines.LINES,
                                       new VzLines.Style(Color.yellow, 2)));
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
