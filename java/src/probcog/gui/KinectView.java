package probcog.gui;

import java.awt.*;
import java.io.*;
import java.util.*;
import javax.swing.JFrame;

import lcm.lcm.*;

import april.config.*;
import april.jmat.LinAlg;
import april.vis.*;
import april.util.*;

import probcog.lcmtypes.*;
import probcog.perception.*;
import probcog.sensor.*;
import probcog.util.*;

// XXX If we're passing around objects with their bounding boxes over LCM,
// is there any reason for this to be running a full tracker?
/** Just a windowed view of the raw kinect data with some object box overlays */
public class KinectView implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();

    VisWorld visWorld;
    VisLayer visLayer;

    private KinectSensor kinect;

    private Timer updateTimer;
    private static final int UPDATE_RATE = 10; // # updates per second

    // Observations for bounding boxes
    ExpiringMessageCache<observations_t> observations =
        new ExpiringMessageCache<observations_t>(2.5, true);
        
        ArrayList<double[]> points;

    public KinectView(Config config_, VisWorld vw, VisLayer vl) throws IOException
    {
        // Make a goddamn kinect
        kinect = new KinectSensor(config_);

        // Initialize the image frame and canvas
        visWorld = vw;

        visLayer = vl;

        //Set up initial camera view
        visLayer.cameraManager.uiLookAt(new double[] {1, 0, .7},// Camera position
                                  new double[] {0, 0, 0},// Point looking at
                                  new double[] {0, 0, 1},// Up
                                  false);

		class RefreshTask extends TimerTask {
			public void run() {
				updateKinectInfo();
                redrawKinectData();
			}
    	}
		updateTimer = new Timer();
		updateTimer.schedule(new RefreshTask(), 1000, 1000/UPDATE_RATE);
    }
    
    public void updateKinectInfo(){
    	if(kinect.stashFrame()){
            points = Util.extractPoints(kinect);
    	}
    }
    
    public VisWorld getVisWorld(){
    	return visWorld;
    }

    private void redrawKinectData()
    {
    	VisWorld.Buffer buffer = visWorld.getBuffer("kinect");

		if(points != null && points.size() > 0){
			VisColorData colors = new VisColorData();
			VisVertexData vertexData = new VisVertexData();
			for(double[] pt : points){
				vertexData.add(new double[]{pt[0], pt[1], pt[2]});

                // Flip RGB
    			colors.add(ColorUtil.swapRedBlue((int)pt[3]));
			}
			VzPoints visPts = new VzPoints(vertexData, new VzPoints.Style(colors, 2));
			buffer.addBack(visPts);
		}
		buffer.swap();
    }

    // === LCM =====================
    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ioex) {
            System.err.println("ERR: Error handling LCM channel - "+channel);
            ioex.printStackTrace();
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
        throws IOException
    {
        if (channel.equals("OBSERVATIONS")) {
            // Handle observations
            observations_t obs = new observations_t(ins);
            observations.put(obs, obs.utime);
        }
    }
}
