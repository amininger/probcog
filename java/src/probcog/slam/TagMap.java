package probcog.slam;

import java.io.*;
import java.util.*;

import april.util.*;

public class TagMap
{
    public GridMap gm = GridMap.makeMeters(0,0,1,1,0.05,0);
    public ArrayList<TagXYT> tags = new ArrayList<TagXYT>();

    public static class TagXYT
    {
        public int id;
        public double[] xyt;

        public TagXYT(int id, double[] xyt)
        {
            this.id = id;
            this.xyt = xyt;
        }
    }

    public TagMap(GridMap gm, ArrayList<TagXYT> tags)
    {
        this.gm = gm;
        this.tags = tags;
    }

    static TagMap load(StructureReader fin) throws IOException
    {
        double x0 = fin.readDouble();
        double y0 = fin.readDouble();
        int w = fin.readInt();
        int h = fin.readInt();
        double mpp = fin.readDouble();
        byte[] data = fin.readBytes();

        GridMap gm = GridMap.makePixels(x0, y0, w, h, mpp, 0, data);

        ArrayList<TagXYT> tags = new ArrayList<TagXYT>();
        int n = fin.readInt();
        for (int i = 0; i < n; i++) {
            fin.blockBegin();
            int id = fin.readInt();
            double[] xyt = fin.readDoubles();
            fin.blockEnd();

            tags.add(new TagXYT(id, xyt));
        }

        System.out.printf("Loaded map [%f %f %d %d %f] w/ %d tags\n",
                          x0, y0, w, h, mpp, tags.size());

        return new TagMap(gm, tags);
    }
}
