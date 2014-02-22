package magic.slam;

import java.util.*;
import java.awt.Color;

import april.jmat.*;
import april.util.*;
import april.config.*;
import april.vis.*;

import lcm.lcm.*;

import magic.util.RobotUtil;
import magic.util.GridMapUtil;
import magic.util.AX12PoseLidar;
import magic.slam.TerrainMapper.Sweep;
import magic.slam.TerrainMapper.Slice;

/** This implementation of HazardMap is intended for INDOOR USE ONLY.
 *  Naively assumes that the world is a plane and that the robot is
 *  restricted to driving in this plane. This prevents the robot from
 *  doing things such as driving off stages, down stairs,
 *  or getting confused by the CSE Atrium but also limits the types of
 *  terrain the robot may on.
 *
 *  Use a simple message passing scheme to determine relevant driveable
 *  locations. Messages passed between cells hypothesize on what could be
 *  in that void. Longer spaces between cells result in larger potential
 *  drops that could kill the robot, and thus we begin to mark things as
 *  being dangerous. We need to be relatively optimistic without actively
 *  filling in garbage like the spiral staircase as "probably OK"
 *
 *  Option (1): Look for relatively flat regions connected to the
 *  robot using UF. (Difficulty: what plane is the robot connected to?)
 *
 *  Option (2): Just find relative flat regions and call them driveable. The
 *  robot will not be able to reach the others.
 *
 *  Option (3): Determine robot plane from RANSAC or similar. Uses MAGIC
 *  obstacle detection, still. Negative obstacles detected by casting
 *  rays to the points actually seen and determining if they must have hit
 *  somewhere below the plane. If yes, mark the region they SHOULD have hit
 *  as an obstacle.
 */
public final class FlatWorldHazardMap implements HazardMap
{
    static final class Cell
    {
        double zmin = Double.MAX_VALUE, zmax = -Double.MAX_VALUE;
        boolean observed;
        boolean boundary;
        boolean ground;
    }

    static final class Plane
    {
        double[] p0;
        double[] p1;
        double[] p2;
        double[] n;

        public Plane(double[] p0, double[] p1, double[] p2)
        {
            this.p0 = LinAlg.copy(p0);
            double[] v1 = LinAlg.subtract(p1, p0);
            double[] v2 = LinAlg.subtract(p2, p0);
            this.n = LinAlg.normalize(LinAlg.crossProduct(v1, v2));

            // Debugging only
            this.p1 = LinAlg.copy(p1);
            this.p2 = LinAlg.copy(p2);

            if (this.n[2] < 0) {
                this.n[0] *= -1;
                this.n[1] *= -1;
                this.n[2] *= -1;
            }
        }

        /** Distance to plane from point */
        public double distance(double[] q)
        {
            if (q == null)
                return 0;

            return Math.abs(LinAlg.dotProduct(LinAlg.subtract(q, p0), n));
        }

        /** Distance a ray must travel from its origin to hit the plane */
        public double rayDistance(double[] l0, double[] dir)
        {
            double denom = LinAlg.dotProduct(dir, n);

            // Parallel line
            if (MathUtil.doubleEquals(denom, 0))
                return Double.MAX_VALUE;

            return LinAlg.dotProduct(LinAlg.subtract(p0, l0), n)/denom;
        }

        public void render(VisWorld vw)
        {
            VisWorld.Buffer vb = vw.getBuffer("plane-fit");
            vb.setDrawOrder(100);

            VzRectangle surface = new VzRectangle(20,20, new VzMesh.Style(new Color(0x55FF0000, true)));

            double[] quat = LinAlg.quatCompute(new double[] {0,0,1}, n);
            double[] quat_ = LinAlg.quatInverse(quat);
            vb.addBack(new VisChain(LinAlg.translate(p0),
                                    LinAlg.quatToMatrix(quat),
                                    //LinAlg.translate(p0),
                                    surface));
            ArrayList<double[]> planePoints = new ArrayList<double[]>();
            planePoints.add(p0);
            planePoints.add(p1);
            planePoints.add(p2);
            vb.swap();

            vb = vw.getBuffer("plane-points");
            vb.setDrawOrder(200);
            vb.addBack(new VzPoints(new VisVertexData(planePoints),
                                    new VzPoints.Style(Color.yellow,4)));

            vb.swap();

        }
    }

    Config config = RobotUtil.getConfig();

    Cell cells[];
    double x0, y0;
    double metersPerPixel;
    int width, height;

    ArrayList<double[]> points = new ArrayList<double[]>();

    // we allow traversable ground to have this slope without
    // penalty.  Distance correponds to metersPerPixel. I.e.,
    // dz = slope * metersPerPixel.
    // XXX only works for 4-neighbors.
    double dz;

    // provable Z height difference between two adjacent cells that
    // makes a cell a boundary.
    double maxJump;

    // provable Z height difference within a cell that makes me a
    // boundary.
    double dzObstacle;

    // information passing iterations
    int niters;

    // Ground filtering height
    double robotZ;

    // Laser Data
    Sweep sweep;

    // Debugging
    VisWorld vw;
    boolean debug;

    public FlatWorldHazardMap(double x0, double y0, double sizex, double sizey, double metersPerPixel,
                              double maxJump, double dzObstacle, int niters,
                              double robotZ, Sweep sweep, VisWorld vw)
    {
        this.x0 = x0;
        this.y0 = y0;

        this.metersPerPixel = metersPerPixel;
        width = (int) (sizex / this.metersPerPixel + 1);
        height = (int) (sizey / this.metersPerPixel + 1);

        cells = new Cell[width*height];
        for (int i = 0; i < cells.length; i++) {
            cells[i] = new Cell();
        }

        this.maxJump = maxJump;
        this.dzObstacle = dzObstacle;
        this.niters = niters;
        dz = 0.25 * metersPerPixel;

        this.robotZ = robotZ;
        this.sweep = sweep;

        this.config = config;
        this.vw = vw;
        if (vw != null)
            debug = true;
        else
            debug = false;
    }

    public void addPoint(double x, double y, double z)
    {
        int ix = (int) ((x - x0) / metersPerPixel);
        int iy = (int) ((y - y0) / metersPerPixel);

        if (ix < 0 || iy < 0 || ix >= width || iy >= height)
            return;

        Cell cell = cells[iy*width+ix];

        cell.zmin = Math.min(cell.zmin, z);
        cell.zmax = Math.max(cell.zmax, z);

        // Filter out points that could potentially be ground and save them.
        double[] orig = new double[] {x0 + (width/2)*metersPerPixel,
                                      y0 + (height/2)*metersPerPixel};
        if (!cell.observed &&
            Math.abs(robotZ - z) < 0.20 &&
            LinAlg.distance(orig, new double[] {x,y}) < 5.0)
        {
            cell.observed = true;
            points.add(new double[] {x,y,z});
        }

    }

    void doMessage(Cell ca, Cell cb)
    {
        // update ca given cb.
        if (ca.observed)
            return;

        // cb has nothing to tell us (yet)
        if (cb.zmin == Double.MAX_VALUE)
            return;

        // if ca has no data, initialize us from our neighbor.
        if (ca.zmin == Double.MAX_VALUE) {
            ca.zmin = cb.zmin - dz;
            ca.zmax = cb.zmax + dz;
        } else {
            ca.zmin = Math.min(ca.zmin, cb.zmin - dz);
            ca.zmax = Math.max(ca.zmax, cb.zmax + dz);
        }
    }

    public void compute()
    {
        // Find the ground
        // (1) Fit a plane to the ground data
        // (2) Mark all of the points on the plane
        //
        // To render the grid map...
        // (a) Dilate the drivable points on the grid map
        // (b) Dilate the rest of the non-drivable points the same amount

        if (points.size() < 25) {
            // XXX What's a more robust way to deal with this? Really, this should
            // mean life isn't safe
            System.out.printf("Insufficient ground points. Only %d found\n", points.size());
        }

        // Use RANSAC to fit plane to potential ground points
        Random rand = new Random(59871941);
        int ransacIters = 100;
        Plane bestPlane = null;
        int bestScore = -1;
        double maxDist = 0.03;
        for (int i = 0; i < ransacIters && points.size() >= 25; i++) {
            double[] p0 = points.get(rand.nextInt(points.size()));
            double[] p1 = points.get(rand.nextInt(points.size()));
            double[] p2 = points.get(rand.nextInt(points.size()));

            if (Arrays.equals(p0, p1) || Arrays.equals(p0, p2) || Arrays.equals(p1, p2))
                continue;

            // Construct a plane and have points vote
            Plane plane = new Plane(p0, p1, p2);
            int score = 0;
            for (double[] q: points) {
                if (plane.distance(q) < maxDist)
                    score++;
            }

            if (score > bestScore) {
                bestScore = score;
                bestPlane = plane;
            }
        }

        // Default plane...Z is straight up
        if (bestPlane == null) {
            bestPlane = new Plane(new double[] {0,0,robotZ}, new double[] {1,0,robotZ}, new double[] {0,1,robotZ});
        }

        // Debugging
        if (debug) {
            bestPlane.render(vw);
        }

        // Mark points on plane
        for (double[] q: points) {
            if (bestPlane.distance(q) < maxDist) {
                int ix = (int) ((q[0] - x0) / metersPerPixel);
                int iy = (int) ((q[1] - y0) / metersPerPixel);

                cells[iy*width + ix].ground = true;
            }
        }

        ArrayList<double[]> linesUp = new ArrayList<double[]>();
        ArrayList<double[]> linesDown = new ArrayList<double[]>();

        // Recast laser rays. If they fall SHORT of the ground plane by a
        // sufficient distance, mark the actual bin as an obstacle. If they
        // go LONG of the ground plane by a sufficient distance, mark the
        // place they SHOULD have hit as an obstacle
        double rPct = .85;
        for (int sliceidx = 0; sliceidx < sweep.slices.size(); sliceidx++) {
            Slice slice = sweep.slices.get(sliceidx);
            //System.out.printf("Num slices: %d\n", sweep.slices.size());

            // Body to local frame
            double B2L[][] = LinAlg.quatPosToMatrix(slice.data.pose.orientation, slice.data.pose.pos);

            // Sensor to body transform
            double[][] S2B = LinAlg.multiplyMany(ConfigUtil.getRigidBodyTransform(config, "HOKUYO_LIDAR"),
                                                 LinAlg.rotateY(-slice.data.status.position_radians),
                                                 LinAlg.translate(0,0,config.requireDouble("HOKUYO_LIDAR.zoffset")));

            double[][] S2L = LinAlg.multiplyMany(B2L, S2B);

            for (int i = 0; i < slice.data.laser.nranges; i++) {
                double r = slice.data.laser.ranges[i];

                // For now, let's ignore out-of-range and other errors
                if (r < 0 || r > 20)
                    continue;

                double theta = slice.data.laser.rad0 + slice.data.laser.radstep*i;

                // Construct ray representation. Assuming our ground plane is
                // accurate, where SHOULD our laser point have hit the ground
                // given the robot's scanning position.
                double[] l0 = new double[3];
                l0 = LinAlg.transform(S2L, l0);

                double[] dir = new double[3];
                double ct = Math.cos(theta);
                double st = Math.sin(theta);
                dir[0] = r*ct;
                dir[1] = r*st;
                dir = LinAlg.normalize(LinAlg.subtract(LinAlg.transform(S2L, dir), l0));

                double hypR = bestPlane.rayDistance(l0, dir);
                //System.out.println(hypR);

                // Case 1: Came up short. Mark actual r
                // Case 2: Within tolerance. Nothing to worry about
                // Case 3: Went too far. Mark hypR

                if (hypR < 0)   // What about super large hypR?
                    continue;
                //System.out.printf("[%f,%f,%f] - [%f,%f,%f]\n", l0[0], l0[1], l0[2], dir[0], dir[1], dir[2]);

                boolean up = false;
                double[] xyz = new double[3];
                if (r < hypR*rPct) {
                    continue; // XXX Leave "up" to the up detector
                    // Case 1: Positive obstacle
                    //xyz[0] = r*ct;
                    //xyz[1] = r*st;
                    //up = true;
                } else if (r*rPct > hypR) {
                    // Case 3: Negative obstacle
                    xyz[0] = hypR*ct;
                    xyz[1] = hypR*st;
                } else {
                    // Case 2: Within tolerance
                    continue;
                }
                // These points must actually be projected into the robot local
                // frame for mapbuilding. Filter out anything above Z meters
                xyz = LinAlg.transform(S2L, xyz);


                double zDiff = bestPlane.distance(xyz);
                // Obstacle too tall
                if (up && xyz[2] - robotZ > 1.2)
                    continue;

                // Up obstacle too small.
                if (up && zDiff < 0.10)
                    continue;

                if (debug) {
                    if (up) {
                        linesUp.add(l0);
                        linesUp.add(xyz);
                    } else {
                        linesDown.add(l0);
                        linesDown.add(xyz);
                    }
                }

                int ix = (int) ((xyz[0] - x0) / metersPerPixel);
                int iy = (int) ((xyz[1] - y0) / metersPerPixel);
                if (ix < 0 || iy < 0 || ix >= width || iy >= height)
                    continue;

                cells[iy*width + ix].boundary = true;
            }
        }

        if (debug) {
            vw.getBuffer("rays-up").addBack(new VzLines(new VisVertexData(linesUp),
                                                        VzLines.LINES,
                                                        new VzLines.Style(Color.red, 1)));
            vw.getBuffer("rays-up").swap();
            vw.getBuffer("rays-down").addBack(new VzLines(new VisVertexData(linesDown),
                                                        VzLines.LINES,
                                                        new VzLines.Style(Color.yellow, 1)));
            vw.getBuffer("rays-down").swap();
        }

        // XXX Classic "up" detection
        final int neighborsDown[][] = new int[][] { { 0, 1 },
                                              { 1, 0 } };

        final int neighborsUp[][] = new int[][] { { 0, -1 },
                                            { -1, 0 } };

        // We think 2 iterations is enough for all cases (information
        // propagates to all other cells)
        for (int iters = 0; iters < niters; iters++) {

            if (true)
            {
                for (int iy = 1; iy+1 < height; iy++) {
                    for (int ix = 1; ix+1 < width; ix++) {

                        Cell ca = cells[iy*width + ix];

                        for (int n = 0; n < neighborsDown.length; n++) {
                            int nx = ix + neighborsDown[n][0];
                            int ny = iy + neighborsDown[n][1];

                            Cell cn = cells[ny*width + nx];

                            // pass information between (ix,iy) and (nx, ny)
                            doMessage(ca, cn);
                            doMessage(cn, ca);
                        }
                    }
                }
            }
            if (true)
            {
                // same thing, backwards propagation order.
                for (int iy = height-2; iy >= 1; iy--) {
                    for (int ix = width-2; ix >= 1; ix--) {
                        Cell ca = cells[iy*width + ix];

                        for (int n = 0; n < neighborsUp.length; n++) {
                            int nx = ix + neighborsUp[n][0];
                            int ny = iy + neighborsUp[n][1];

                            Cell cn = cells[ny*width + nx];

                            // pass information between (ix,iy) and (nx, ny)
                            doMessage(ca, cn);
                            doMessage(cn, ca);
                        }
                    }
                }
            }
        }

        final int neighbors[][] = new int[][] { { 0, 1 },
                                                { 1, 0 },
                                                { 0, -1 },
                                                { -1, 0 } };

        // label pixels as boundaries if they must be taller than their neighbors.
        for (int iy = 1; iy+1 < height; iy++) {
            for (int ix = 1; ix+1 < width; ix++) {
                Cell me = cells[iy*width + ix];

                if (me.observed && me.zmax - me.zmin > dzObstacle) {
                    me.boundary = true;
                    continue;
                }

                for (int n = 0; n < neighbors.length; n++) {
                    int nx = ix + neighbors[n][0];
                    int ny = iy + neighbors[n][1];

                    Cell cn = cells[ny*width + nx];

                    if ((me.zmin - cn.zmax) > maxJump) {
                        me.boundary = true;
                        break;
                    }
                }
            }
        }
    }

    public GridMap makeGridMap()
    {
        GridMap gm = GridMap.makePixels(x0, y0, width, height, metersPerPixel, 0, true);

        // Put walls back in place
        for (int iy = 0; iy < height; iy++) {
            for (int ix = 0; ix < width; ix++) {
                Cell c = cells[iy*width+ix];

                if (c.ground)
                    gm.data[iy*gm.width + ix] = (byte) 127;

                if (c.boundary)
                    gm.data[iy*gm.width + ix] = (byte) 255;

            }
        }

        return gm;
    }
}
