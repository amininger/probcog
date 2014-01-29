package probcog.sensor;

import java.awt.Color;
import java.awt.Rectangle;
import java.awt.image.BufferedImage;
import java.awt.image.DataBufferByte;
import java.util.ArrayList;
import java.util.Collections;
import java.util.Comparator;
import java.util.List;

import javax.swing.JFrame;

import probcog.sim.SimLocation;
import probcog.sim.SimObjectPC;
import april.jmat.LinAlg;
import april.jmat.geom.GRay3D;
import april.sim.Collisions;
import april.sim.SimObject;
import april.sim.SimWorld;
import april.util.JImage;
import april.util.TimeUtil;
import april.vis.DefaultCameraManager;
import april.vis.VisCameraManager.CameraPosition;
import april.vis.VisCanvas;
import april.vis.VisChain;
import april.vis.VisLayer;
import april.vis.VisWorld;

/** Provides KinectSensor-like access to the 3D world. Can return
 *  colors and points for objects in the world. Points that do not
 *  make up objects are automatically returned as transparent points
 *  at the origin.
 */
public class SimKinectSensor implements Sensor
{
	public class SimPixel{
		public SimObject target;
		public double[] point;
		public SimPixel(){
			target = null;
			point = new double[4];
		}
	}
	
    // Sim Kinect parameters
    private static final int NUM_SUBREGIONS = 12; // To increase/decrease the resolution I recommend tuning this parameter only
    private static final int GRID_DEPTH = 3;
    public static final int WIDTH = (int)(4*Math.pow(2, GRID_DEPTH) * NUM_SUBREGIONS);
    public static final int HEIGHT = (int)(3*Math.pow(2, GRID_DEPTH) * NUM_SUBREGIONS);
    public static final double HFOV = 57.0;
    public static final double VFOV = 43.0;
    public static final double POS_NOISE = .001;
    public static final int COLOR_NOISE = 2;

    CameraPosition camera = new CameraPosition();

    SimWorld sw;

    JFrame jf;
    JImage jim;
    VisWorld vw;
    VisLayer vl;
    VisCanvas vc;

    /** Takes as input the SimWorld from which point data is generated */
    public SimKinectSensor(SimWorld sw_)
    {
        if (false) {
            jf = new JFrame("DEBUG WINDOW");
            jf.setSize(WIDTH,HEIGHT);
            jim = new JImage(WIDTH,HEIGHT);
            jim.setFlipY(true);
            jf.add(jim);
            //jf.add(vc);
            jf.setVisible(true);
        }

        sw = sw_;

        vw = new VisWorld();
        vl = new VisLayer(vw);

        // Set up the kinect view. Anchored to a fixed point. Sets this up
        // as our VisLayer's default view, too, so when we render for the
        // canvas, our view of the world will be accurate
        camera.eye = new double[] {0.6, 0, 1.0};    // Camera position
        camera.lookat = new double[3];              // Looks at origin
        camera.up = new double[] {-1.0, 0, 1.0};    // Up vector
        //camera.eye = new double[] {1, 1, 1};
        //camera.lookat = new double[] {0, 0, 0};
        //camera.up = new double[] {-1, -1, 1};

        camera.perspective_fovy_degrees = VFOV;
        camera.layerViewport = new int[] {0,0,WIDTH,HEIGHT};

        DefaultCameraManager cm = new DefaultCameraManager();

        cm.UI_ANIMATE_MS = 0;
        cm.BOOKMARK_ANIMATE_MS = 0;
        cm.FIT_ANIMATE_MS = 0;
        cm.interfaceMode = 3.0;

        vl.cameraManager = cm;
        vl.cameraManager.goBookmark(camera);
        vl.backgroundColor = Color.white;

        vc = new VisCanvas(vl);
        vc.setSize(WIDTH, HEIGHT);

        (new RenderThread()).start();
    }

    class RenderThread extends Thread
    {
        int Hz = 30;
        public void run()
        {
            while (true) {
                VisWorld.Buffer vb = vw.getBuffer("objs");
                for (SimObject obj: sw.objects) {
                    vb.addBack(new VisChain(obj.getPose(),
                                            obj.getVisObject()));
                }
                vb.swap();
                // I don't feel like this guarantees that we'll have our image data in
                // time for color sampling...
                vc.draw();
                TimeUtil.sleep(1000/Hz);
            }
        }
    }

    /** Return the real-world position of the camera */
    public double[][] getCameraXform()
    {
        double[][] camMatrix = new double[4][4];
        // Kinect axes
        double[] z = LinAlg.normalize(LinAlg.subtract(camera.lookat, camera.eye));
        double[] y = LinAlg.normalize(LinAlg.scale(camera.up, -1));
        double[] x = LinAlg.normalize(LinAlg.crossProduct(y, z));
        y = LinAlg.normalize(LinAlg.crossProduct(z, x));

        camMatrix[0][0] = x[0];
        camMatrix[0][1] = y[0];
        camMatrix[0][2] = z[0];
        camMatrix[0][3] = camera.eye[0];
        camMatrix[1][0] = x[1];
        camMatrix[1][1] = y[1];
        camMatrix[1][2] = z[1];
        camMatrix[1][3] = camera.eye[1];
        camMatrix[2][0] = x[2];
        camMatrix[2][1] = y[2];
        camMatrix[2][2] = z[2];
        camMatrix[2][3] = camera.eye[2];
        camMatrix[3][3] = 1.0;

        return camMatrix;
    }
    
    /*********
     * scanRegions(boolean[][] higherGrid, int depth, int maxDepth, SimPixel[] pixels)
     * 
     * Iterates over each region specified by the given grid that are marked true
     * For each region, it splits it into 4 subregions and scans each one
     * If a region contains an object, it marks it for further scanning in the lowerGrid
     * and recurses until the depth = 1 (lowest)
     */
    public void scanRegions(boolean[][] higherGrid, int depth, int maxDepth, SimPixel[] pixels){
    	int numRows = higherGrid.length;
    	int numCols = higherGrid[0].length;
    	boolean[][] lowerGrid = new boolean[numRows*2][numCols*2];
    	int subregionWidth = WIDTH/numCols/2;
    	int subregionHeight = HEIGHT/numRows/2;
    	Rectangle subregion = new Rectangle(0, 0, subregionWidth, subregionHeight);
    	for(int r = 0; r < higherGrid.length; r++){
    		for(int c = 0; c < higherGrid[0].length; c++){
    			if(!higherGrid[r][c]){
    				continue;
    			}
    			for(int i = 0; i < 2; i++){
    				for(int j = 0; j < 2; j++){
    					subregion.y = (2*r+i) * subregionHeight;
    					subregion.x = (2*c+j) * subregionWidth;
    					if(scanRegion(subregion, depth, maxDepth, pixels)){
    						markRegionsToScan(lowerGrid, 2*r+i, 2*c+j);
    					}
    				}
    			}
    		}
    	}
    	if(depth > 1){
    		scanRegions(lowerGrid, depth-1, maxDepth, pixels);
    	} else {
    		scanRemaining(lowerGrid, pixels);
    	}
    }
    
    /*********
     * boolean scanRegion(Rectangle region, int depth, int maxDepth, SimPixel[] pixels)
     * 
     * Scans the given region on the screen (given in pixel coordinates) and populates the given pixels array
     * with the results of the ray tracing
     * It uses the depth and max depth to determine how coarse of a scan to do, 
     * I.e. a depth of 1 means a fine scan, a depth of 4 is coarser
     */
    public boolean scanRegion(Rectangle region, int depth, int maxDepth, SimPixel[] pixels){
    	boolean hitObject = false;
    	int startX = region.x + (int)Math.pow(2, depth-1) - 1;
    	int startY = region.y + (int)Math.pow(2, depth-1) - 1;
    	int endX = region.x + region.width;
    	int endY = region.y + region.height;
    	int stride = (int)Math.pow(2, depth);
    	//System.out.println("REG:" + startX + ", " + startY + ", " + endX + ", " + endY + ", " + stride);
    	for(int x = startX; x < endX; x += stride){
    		for(int y = startY; y < endY; y += stride){
    			hitObject = hitObject | scanPoint(x, y, pixels);
    		}
    	}
    	return hitObject;
    }
    
    /*********
     * void markRegionsToScan(boolean[][] grid, int row, int col)
     * 
     * Marks the given row/col in the grid as true, as well as all the neighbors (8-connected)
     */
    public void markRegionsToScan(boolean[][] grid, int row, int col){
    	int numRows = grid.length;
    	int numCols = grid[0].length;
    	
    	for(int r = row-1; r <= row+1; r++){
    		for(int c = col-1; c <= col+1; c++){
    			if(r >= 0 && c >= 0 && r < numRows && c < numCols){
    				grid[r][c] = true;
    			}
    		}
    	}
    }
    
    /**********
     * void scanRemaining(boolean[][] grid, SimPixel[] pixels)
     * 
     * This iterates over all regions marked as true in the given grid
     * In each region, it iterates over all pixels and scans them if they have not yet been scanned
     */
    public void scanRemaining(boolean[][] grid, SimPixel[] pixels){
    	int numRows = grid.length;
    	int numCols = grid[0].length;
    	int regionWidth = WIDTH/numCols;
    	int regionHeight = HEIGHT/numCols;
    	for(int r = 0, startY = 0; r < numRows; r++, startY += regionHeight){
    		for(int c = 0, startX = 0; c < numCols; c++, startX += regionWidth){
    			if(grid[r][c] == false){
    				continue;
    			}
    			
    			int endX = startX + regionWidth;
    			int endY = startY + regionHeight;
    			for(int x = startX; x < endX; x++){
    				for(int y = startY; y < endY; y++){
    					int i = y * WIDTH + x;
    					if(pixels[i] == null){
    						scanPoint(x, y, pixels);
    					}
    				}
    			}
    		}
    	}
    }

    
    
    /**********
     * boolean scanPoint(int px, int py, SimPixel[] pixels)
     * 
     * Scans the scene at the given point (in pixel space) using ray tracing from the simulated kinect
     * If it hits an object, it adds it at the appropriate index in the supplied array and returns true
     * If it does not hit anything, it returns false and puts an empty SimPixel in the array (target = null and point = zero)
     */
    int numScans = 0;
    public boolean scanPoint(int px, int py, SimPixel[] pixels){
		int i = py * WIDTH + px;
		numScans++;
		pixels[i] = getPixel(px, py);
    	if(pixels[i].target == null){
    		// Ray didn't hit anything
    		//pixels[i].point = new double[4];
    		return false;
    	}
    	if(pixels[i].target instanceof SimLocation){
    		pixels[i].target = null;
    		// We aren't segmenting sim locations
    		return false;
    	}
		return true;
    }
    
    // returns a random number between -1 and 1, more biased towards 0
    private double getNoiseScale(){
    	double rand = (Math.random() - .5) * 2; // -1 to 1
    	rand = Math.signum(rand) * rand * rand;  // square to give it more of a curve
    	return rand;
    }
    
    public SimPixel getPixel(int ix, int iy){
    	SimPixel pixel = new SimPixel();
    	pixel.target = null;
    	
    	// XXX Infinite loop possibility
        BufferedImage im;
        do {
            im = vc.getLatestFrame();
            if (im == null)
                TimeUtil.sleep(10);
        } while (im == null);

        if (false) {
            jim.setImage(im);
        }

        // XXX Currently only supports 3BYTE BGR
        byte[] buf = ((DataBufferByte)(im.getRaster().getDataBuffer())).getData();
        //System.out.println(im.getWidth() + " " + im.getHeight() + " " + buf.length);

        // Find the ray leaving the camera
        GRay3D ray = camera.computeRay(ix, iy);

        // Search through the objects in the world and try to find the first
        // point at which the ray collides with one of these objects.
        // Apply color as needed, afterwards, defaulting otherwise to white.
        // If there is no point found, return all zeros
        double minDist = Double.MAX_VALUE;
        SimObject minObj = null;
        synchronized (sw) {
            for (SimObject obj: sw.objects) {
                double dist = Collisions.collisionDistance(ray.getSource(),
                                                           ray.getDir(),
                                                           obj.getShape(),
                                                           obj.getPose());
                if (dist < minDist) {
                    minDist = dist;
                    minObj = obj;
                }
            }
        }
        
        if(minObj == null){
        	return pixel;
        } else if(minDist >= 8.0){
        	return pixel;
        } else {
        	pixel.target = minObj;
        }
//        pixel.target = minObj;
//        if(minObj == null) {
//            double[] xyFloor = ray.intersectPlaneXY();
//            minDist = LinAlg.distance(ray.getSource(), xyFloor);
//        }

        // Compute the point in space we collide with the object at
        double[] xyzc = ray.getPoint(minDist);
        xyzc = LinAlg.resize(xyzc, 4);
        
        // Add random noise
        for(int i = 0; i < 3; i++){
        	xyzc[i] += getNoiseScale() * POS_NOISE;
        }
        
        // If the object in question supports a color query, assign it
        // a color. Otherwise, default to white.
        int idx = iy*WIDTH + ix;
        int b = buf[3*idx + 0];
        int g = buf[3*idx + 1];
        int r = buf[3*idx + 2];
        if(b < 0){
            b = LinAlg.clamp(b + (int)(getNoiseScale() * COLOR_NOISE), -128, -1);
        } else {
            b = LinAlg.clamp(b + (int)(getNoiseScale() * COLOR_NOISE), 0, 127);
        }
        if(g < 0){
            g = LinAlg.clamp(g + (int)(getNoiseScale() * COLOR_NOISE), -128, -1);
        } else {
            g = LinAlg.clamp(g + (int)(getNoiseScale() * COLOR_NOISE), 0, 127);
        }
        if(r < 0){
            r = LinAlg.clamp(r + (int)(getNoiseScale() * COLOR_NOISE), -128, -1);
        } else {
            r = LinAlg.clamp(r + (int)(getNoiseScale() * COLOR_NOISE), 0, 127);
        }
        xyzc[3] = 0xff000000 |
                  (b & 0xff) |
                  (g & 0xff) << 8 |
                  (r & 0xff) << 16;
        
        pixel.point = xyzc;
        return pixel;
    }

    /** Get the RGBXYZ point corresponding to virtual kinect pixel (ix,iy) */
    public double[] getXYZRGB(int ix, int iy)
    {
    	SimPixel pixel = getPixel(ix, iy);
    	return pixel.point;
    }
    
    public SimPixel[] getAllPixels(){
    	SimPixel[] pixels = new SimPixel[WIDTH*HEIGHT];
    	numScans = 0;    
    	
    	// AM: Optimization here, tries to do a coarse initial scan over the scene, only doing
    	//      finer scans when an object is hit. see scanSceneSubset for more details
    	// Empirical testing on the simulator shows a 8-10 times speedup
    	int numRegions = NUM_SUBREGIONS;
		boolean[][] grid = new boolean[numRegions][numRegions];
		for(int r = 0; r < numRegions; r++){
			for(int c = 0; c < numRegions; c++){
				grid[r][c] = true;
			}
		}
		
    	boolean scanSubsets = true;
    	if(scanSubsets){
    		scanRegions(grid, GRID_DEPTH, GRID_DEPTH, pixels);
    	} else {
    		// Scan Whole Scene (very slow)
    		scanRegions(grid, 1, 1, pixels);
    	}
    	
    	return pixels;
    }
    
    public BufferedImage getImage(){
    	ArrayList<double[]> points = getAllXYZRGB();
    	BufferedImage image = new BufferedImage(WIDTH, HEIGHT, BufferedImage.TYPE_INT_RGB);
    	for(int y = 0; y < HEIGHT; y++){
    		for(int x= 0; x < WIDTH; x++){
    			int loc = y*WIDTH + x;
    			image.setRGB(x, y, (int)points.get(loc)[3]);
    		}
    	}
    	return image;    	
    }
    
    public ArrayList<double[]> getAllXYZRGB(){
    	SimPixel[] pixels = getAllPixels();
    	ArrayList<double[]> points = new ArrayList<double[]>();

    	int i = 0;
        for (int y = 0; y < HEIGHT; y++) {
            for (int x = 0; x < WIDTH; x++, i++) {
            	if(pixels[i] == null){
            		points.add(new double[4]);
            	} else {
            		points.add(pixels[i].point);
            	}
            }
        }
    	return points;
    }


    public int getWidth()
    {
        return WIDTH;
    }

    public int getHeight()
    {
        return HEIGHT;
    }
    
    public boolean stashFrame()
    {
    	return true;
    }
}
