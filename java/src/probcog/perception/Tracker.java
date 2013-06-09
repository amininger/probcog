package probcog.perception;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.jmat.geom.*;
import april.util.*;

import lcm.lcm.*;

import probcog.arm.*;
import probcog.lcmtypes.*;

public class Tracker
{
    static LCM lcm = LCM.getSingleton();

    // Soar stuff
    private Object soarLock;
    private soar_objects_t soar_lcm;

    // Arm Stuff
    private Object armLock = new Object();
    private robot_command_t robot_cmd;

    private KinectSegment segmenter;
    private ArmCommandInterpreter armInterpreter;

    // World state, as decided by Soar and the perception system
    ArrayList<Obj> worldState = new ArrayList<Obj>();

    public Tracker(Config config_) throws IOException
    {
        segmenter = new KinectSegment(config_);
        armInterpreter = new ArmCommandInterpreter(false);  // Debug off

        soarLock = new Object();
        soar_lcm = null;
        new ListenerThread().start();
        new TrackingThread().start();
    }

    public ArrayList<Obj> getWorldState()
    {
        return worldState;
    }

    public void compareObjects()
    {
        ArrayList<Obj> visibleObjects = getVisibleObjects();
        ArrayList<Obj> soarObjects = getSoarObjects();

        // XXX - Need some sort of matching code here.
        //
        // Iterate through our existing objects. If there is an object
        // sharing any single label within a fixed distance of an existing
        // object, take the existing object's ID. If the ID has already
        // been taken by another object, take a new ID
        Set<Integer> idSet = new HashSet<Integer>();
        double thresh = 0.02;
        for (Obj newObj: visibleObjects) {
            boolean matched = false;
            double minDist = Double.MAX_VALUE;
            int minID = -1;

            for (Obj oldObj: worldState) {
                if (idSet.contains(oldObj.getID()))
                    continue;
                double dist = LinAlg.distance(newObj.getCentroid(),
                                              oldObj.getCentroid(),
                                              2);
                if (dist < thresh && dist < minDist) {
                    matched = true;
                    minID = oldObj.getID();
                    minDist = dist;
                }
            }

            if (matched) {
                newObj.setID(minID);
            } else {
                newObj.setID(Obj.nextID());
            }
            idSet.add(newObj.getID());
        }

        worldState = visibleObjects;
    }


    /** Returns a list of objects that the kinect sees on the table. The objects
     *  are returned as pointClouds from the segmenter, and are passed to the
     *  classifiers. The resulting point clouds, their locations, and the
     *  classifications are returned.
     **/
    private ArrayList<Obj> getVisibleObjects()
    {
        ArrayList<Obj> visibleObjects = new ArrayList<Obj>();
        boolean assignID = false;

        ArrayList<PointCloud> ptClouds = segmenter.getObjectPointClouds();
        for(PointCloud ptCloud : ptClouds) {
            Obj vObj = new Obj(assignID, ptCloud);
            // XXX - should we be classifying objects before the tracking?
            // XXX - hand off to classifiers
            visibleObjects.add(vObj);
        }

        return visibleObjects;
    }

    /** Returns a list of Obj that Soar believes exists in the world based on
     *  their most recent lcm message. Obj have information such as their center
     *  and features they were classified with. They do not have point clouds.
     **/
    private ArrayList<Obj> getSoarObjects()
    {
        ArrayList<Obj> soarObjects = new ArrayList<Obj>();

        synchronized(soarLock) {
            if(soar_lcm != null) {
                for(int i=0; i<soar_lcm.num_objects; i++) {
                    object_data_t odt = soar_lcm.objects[i];
                    Obj sObj = new Obj(odt.id);
                    double[] xyzrpy = odt.pos;
                    sObj.setCentroid(new double[]{xyzrpy[0], xyzrpy[1], xyzrpy[2]});

                    for(int j=0; j<odt.num_cat; j++) {
                        // XXX - Need to do something with this information
                    }
                    soarObjects.add(sObj);
                }
            }
        }
        return soarObjects;
    }

    // public void sendMessage()
    // {
    //     observations_t obs = new observations_t();
    //     obs.utime = TimeUtil.utime();
    //     synchronized(objectManager.objects) {
    //         obs.click_id = simulator.getSelectedId();
    //     }
    //     obs.sensables = sensableManager.getSensableStrings();
    //     obs.nsens = obs.sensables.length;
    //     obs.observations = classifierManager.getObjectData();
    //     obs.nobs = obs.observations.length;

    //     lcm.publish("TRACKED_OBJECTS",obs);
    // }


    /** Class that continually listens for messages from Soar about what objects
     *  it believes exists in the world. The received lcm message is stored so it
     *  can be used upon request.
     **/
    class ListenerThread extends Thread implements LCMSubscriber
    {
        LCM lcm = LCM.getSingleton();

        public ListenerThread()
        {
            lcm.subscribe("SOAR_OBJECTS", this);
            lcm.subscribe("ROBOT_COMMAND", this);
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(1000/60);    // XXX.
            }
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                messageReceivedEx(lcm, channel, ins);
            } catch (IOException ioex) {
                System.err.println("ERR: LCM channel -"+channel);
                ioex.printStackTrace();
            }
        }

        public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
            throws IOException
        {
            if (channel.equals("SOAR_OBJECTS")) {
                synchronized (soarLock) {
                    soar_lcm = new soar_objects_t(ins);
                }
            } else if (channel.equals("ROBOT_COMMAND")) {
                synchronized (armLock) {
                    robot_cmd = new robot_command_t(ins);
                    armInterpreter.queueCommand(robot_cmd);
                }
            }
        }
    }

    /** Runs in the background, updating our knowledge of the scene */
    public class TrackingThread extends Thread
    {
        public void run()
        {
            while (true) {
                compareObjects();
                ArrayList<Obj> objs = getWorldState();
                synchronized (armLock) {
                    armInterpreter.updateWorld(objs);
                }

                TimeUtil.sleep(1000/30);
            }
        }
    }
}
