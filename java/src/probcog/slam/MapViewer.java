package probcog.slam;

import java.awt.*;
import java.awt.image.*;
import javax.swing.*;
import java.io.*;
import java.util.*;

import april.jmat.*;
import april.util.*;
import april.vis.*;

public class MapViewer
{
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    TagMap map;

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
