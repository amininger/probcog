package probcog.commands;

import java.util.*;

import april.jmat.*;
import april.util.*;

/** A tool for computing wavefront paths through the environment */
public class WavefrontPlanner
{
    GridMap gm;
    float[] costMap;
    double[] start, goal;

    int[][] neighbors = new int[][] {{-1,-1},{0,-1},{1,-1},
                                     {-1,0},        {1,0},
                                     {-1,1}, {0,1}, {1,1}};

    // XXX Debugging w/vis
    /** Construct a path planner for the given map and config space radius. */
    public WavefrontPlanner(GridMap gm, double radius)
    {
        int iters = (int)Math.ceil(radius/gm.metersPerPixel);
        this.gm = gm.dilate((byte)0, iters);
        costMap = new float[gm.data.length];
    }

    private void initMap()
    {
        for (int i = 0; i < gm.data.length; i++)
            costMap[i] = gm.data[i] == 0 ? Float.MAX_VALUE : -1;
        start = null;
        goal = null;
    }

    public float[] getWavefront(double[] s, double[] g)
    {
        initMap();

        // Starting/finishing indices. Note: we search BACK from the goal
        int sx = Integer.MAX_VALUE;
        int sy = Integer.MAX_VALUE;
        int gx = (int) ((g[0]-gm.x0)/gm.metersPerPixel);
        int gy = (int) ((g[1]-gm.y0)/gm.metersPerPixel);
        int w = gm.width;
        int h = gm.height;
        if (s != null) {
            sx = (int) ((s[0]-gm.x0)/gm.metersPerPixel);
            sy = (int) ((s[1]-gm.y0)/gm.metersPerPixel);
        }

        // Error check goal. We only accept plans within our grid map.
        if (gx < 0 || gx >= w ||
            gy < 0 || gy >= h)
            assert (false);

        costMap[gy*w + gx] = 1;

        // Pack indices into heap, weighted by path cost thus far
        IntMaxHeap wavefront = new IntMaxHeap();
        int c = ((gx & 0xffff) << 16) | (gy & 0xffff);
        wavefront.add(c, -costMap[gy*w + gx]);

        // No early termination unless we find goal
        while (wavefront.size() > 0) {
            IntHeapPair ihp = wavefront.removeMaxPair();
            int nx = (ihp.o >> 16) & 0xffff;
            int ny = ihp.o & 0xffff;
            int n = ny*w + nx;

            for (int[] neighbor: neighbors) {
                int npx = nx + neighbor[0];
                int npy = ny + neighbor[1];
                int np = npy*w + npx;

                // Skip out of bounds
                if (npx < 0 || npx >= w || npy < 0 || npy >= h)
                    continue;

                // If it already has a value, skip it as well
                if (costMap[np] > 0)
                    continue;

                // We are assuming fixed-cost movement between all cells (there
                // is no such thing as cell cost weighting), so we can measure
                // cost as being the distance between cells
                float cost = costMap[n] + (float)LinAlg.magnitude(neighbor);
                costMap[np] = cost;
                c = ((npx & 0xffff) << 16) | (npy & 0xffff);
                wavefront.add(c, -cost);

                // If we've reached the start point, terminate
                if (sx == npx && sy == npy) {
                    start = s;
                    goal = g;
                    return costMap;
                }
            }
        }

        assert (s == null); // Otherwise, you asked for the impossible

        // Finished without finding goal. Just send back a cost map
        return costMap;
    }

    // XXX This is sub-optimal, but we split out cost map computation for
    // quick visualization. Only call if we found a path!
    /** Get a path corresponding to the existing cost map */
    public ArrayList<double[]> getPath()
    {
        assert (start != null && goal != null);
        ArrayList<double[]> path = new ArrayList<double[]>();

        // Starting/finishing indices.
        int sx = (int) ((start[0]-gm.x0)/gm.metersPerPixel);
        int sy = (int) ((start[1]-gm.y0)/gm.metersPerPixel);
        int gx = (int) ((goal[0]-gm.x0)/gm.metersPerPixel);
        int gy = (int) ((goal[1]-gm.y0)/gm.metersPerPixel);
        int w = gm.width;
        int h = gm.height;

        path.add(new double[] { gm.x0 + (sx + 0.5)*gm.metersPerPixel,
                                gm.y0 + (sy + 0.5)*gm.metersPerPixel });
        int nx = sx;
        int ny = sy;
        // WARNING: Can loop infinitely if you ask for an impossible path
        while (!(nx == gx && ny == gy)) {
            double min = costMap[ny*w + nx];
            int minx = nx;
            int miny = nx;
            for (int[] neighbor: neighbors) {
                int npx = nx+neighbor[0];
                int npy = ny+neighbor[1];
                float v = costMap[npy*w + npx];
                if (v > 0 && v < min) {
                    min = v;
                    minx = npx;
                    miny = npy;
                }
            }
            assert (minx != nx || miny != ny);
            nx = minx;
            ny = miny;
            double[] n = new double[] { gm.x0 + (nx + 0.5)*gm.metersPerPixel,
                                        gm.y0 + (ny + 0.5)*gm.metersPerPixel };
            path.add(n);
        }

        return path;
    }
}
