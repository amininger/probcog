package probcog.gui;

import java.awt.*;
import java.io.*;
import java.util.*;
import javax.swing.JFrame;

import lcm.lcm.*;

import april.jmat.LinAlg;
import april.vis.*;

import probcog.lcmtypes.kinect_status_t;
import probcog.perception.*;
import probcog.sensor.*;
import probcog.util.Util;

// XXX If we're passing around objects with their bounding boxes over LCM,
// is there any reason for this to be running a full tracker?
/** Just a windowed view of the raw kinect data with some object box overlays */
public class KinectView extends JFrame
{
    VisWorld visWorld;
    VisLayer visLayer;

    private KinectSensor kinect;

    private Timer updateTimer;
    private static final int UPDATE_RATE = 10; // # updates per second

    private Tracker tracker;

    public KinectView(Tracker tracker_)
    {
        tracker = tracker_;

        // Setup Frame
        JFrame frame = new JFrame("Kinect View");
        frame.setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
        frame.setLayout(new BorderLayout());

        // Initialize the image frame and canvas
        visWorld = new VisWorld();

        visLayer = new VisLayer(visWorld);
        VisCanvas visCanvas = new VisCanvas(visLayer);

        //Set up initial camera view
        visLayer.cameraManager.uiLookAt(new double[] {1, 0, .7},// Camera position
                                  new double[] {0, 0, 0},// Point looking at
                                  new double[] {0, 0, 1},// Up
                                  false);


        frame.add(visCanvas, BorderLayout.CENTER);

        // Finalize JFrame
        frame.setSize(400, 300);
        frame.setVisible(true);

		class RefreshTask extends TimerTask{
			public void run() {
				update();
			}
    	}
		updateTimer = new Timer();
		updateTimer.schedule(new RefreshTask(), 1000, 1000/UPDATE_RATE);
    }

    private void update()
    {
    	VisWorld.Buffer buffer = visWorld.getBuffer("objects");

    	synchronized(tracker.stateLock) {
    		for(Obj ob : tracker.getWorldState().values()) {
    			double[][] bbox = ob.getBoundingBox();
                double[] centroid = ob.getCentroid();
                double[] pos = new double[]{centroid[0], centroid[1], centroid[2], 0, 0, 0};
    			double[][] scale = LinAlg.scale(bbox[1][0] - bbox[0][0],
                                                bbox[1][1] - bbox[0][1],
                                                bbox[1][2] - bbox[0][2]);
    			VzBox box = new VzBox(new VzMesh.Style(new Color(0,0,0,0)), new VzLines.Style(Color.cyan, 2));
    			buffer.addBack(new VisChain(LinAlg.translate(pos), scale, box));
    		}
    	}
		buffer.swap();
    }

    private void redrawKinectData()
    {
    	VisWorld.Buffer buffer = visWorld.getBuffer("kinect");
        ArrayList<double[]> points;
        if (!kinect.stashFrame())
            points = new ArrayList<double[]>();
        else
            points = Util.extractPoints(kinect);

		if(points != null && points.size() > 0){
			VisColorData colors = new VisColorData();
			VisVertexData vertexData = new VisVertexData();
			for(double[] pt : points){
				vertexData.add(new double[]{pt[0], pt[1], pt[2]});
    			colors.add((int)pt[3]);
			}
			VzPoints visPts = new VzPoints(vertexData, new VzPoints.Style(colors, 2));
			buffer.addBack(visPts);
		}
		buffer.swap();
    }
}
