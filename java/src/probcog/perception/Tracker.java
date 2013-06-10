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
import probcog.classify.*;
import probcog.classify.Features.FeatureCategory;
import probcog.lcmtypes.*;

public class Tracker
{
    static LCM lcm = LCM.getSingleton();
    // Classification stuff
    ClassifierManager classyManager;

    // Soar stuff
    private Object soarLock;
    private soar_objects_t soar_lcm;

    // Arm Stuff
    private Object armLock = new Object();
    private robot_command_t robot_cmd;

    private KinectSegment segmenter;
    private ArmCommandInterpreter armInterpreter;

    // World state, as decided by Soar and the perception system
    public Object stateLock;
    HashMap<Integer, Obj> worldState = new HashMap<Integer, Obj>();

    public Tracker(Config config_) throws IOException
    {
        segmenter = new KinectSegment(config_);
        classyManager = new ClassifierManager(config_);
        armInterpreter = new ArmCommandInterpreter(false);  // Debug off

        stateLock = new Object();

        soarLock = new Object();
        soar_lcm = null;
        new ListenerThread().start();
        new TrackingThread().start();
    }

    public HashMap<Integer, Obj> getWorldState()
    {
        return worldState;
    }

    // Returns null if object not found
    public Obj getObject(Integer id)
    {
        return worldState.get(id);
    }

    public void compareObjects()
    {
        ArrayList<Obj> visibleObjects = getVisibleObjects();
        ArrayList<Obj> soarObjects = getSoarObjects();

        synchronized (stateLock) {
            HashMap<Integer, Obj> newWorldState = new HashMap<Integer, Obj>();

            // XXX - Need better matching code here.
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

//                for (Obj oldObj: worldState.values()) {
                  for (Obj oldObj: soarObjects) {
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

                newWorldState.put(newObj.getID(), newObj);
            }
            worldState = newWorldState;
        }
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
            vObj.addAllClassifications(classyManager.classifyAll(vObj));
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
                        categorized_data_t cat = odt.cat_dat[j];
                        FeatureCategory fc = Features.getFeatureCategory(cat.cat.cat);
                        Classifications cs = new Classifications();
                        for(int k=0; k<cat.len; k++)
                            cs.add(cat.label[k], cat.confidence[k]);

                        sObj.addClassifications(fc, cs);
                    }
                    soarObjects.add(sObj);
                }
            }
        }
        return soarObjects;
    }

    //////////////////////////////////////////////////////////////
    // Methods for interacting with the classifierManager (used through guis)
    public void clearClassificationData()
    {
        classyManager.clearData();
    }
    public void reloadClassificationData()
    {
        classyManager.reloadData();
    }
    public void undoClassification()
    {
        classyManager.undo();
    }
    public void redoClassification()
    {
        classyManager.redo();
    }
    public void addTraining(FeatureCategory category, ArrayList<Double> features, String label)
    {
        classyManager.addDataPoint(category, features, label);
    }
    public void writeClassificationState(String filename) throws IOException
    {
        classyManager.writeState(filename);
    }
    public boolean canUndoClassification()
    {
        return classyManager.hasUndo();
    }
    public boolean canRedoClassification()
    {
        return classyManager.hasRedo();
    }
    public Classifications getClassification(FeatureCategory category, Obj ob)
    {
        return classyManager.classify(category, ob);
    }


    /** Build up the object_data_t describing the observed objects
     *  in the world. Runs classifiers on the objects and builds
     *  the appropriate lcmtypes to return.
     */
    public object_data_t[] getObjectData()
    {
        object_data_t[] od;
        long utime = TimeUtil.utime();

        int i = 0;
        synchronized (stateLock) {
            od = new object_data_t[worldState.size()];
            for (Obj ob: worldState.values()) {
                od[i] = new object_data_t();
                od[i].utime = utime;
                od[i].id = ob.getID();
                od[i].pos = ob.getPose();
                od[i].bbox = ob.getBoundingBox();

                categorized_data_t[] cat_dat = classyManager.getCategoryData(ob);
                od[i].num_cat = cat_dat.length;
                od[i].cat_dat = cat_dat;

                i++;
            }
        }

        return od;
    }




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
                TimeUtil.sleep(1000/60);
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
                HashMap<Integer, Obj> objs = getWorldState();
                ArrayList<Obj> objsList = new ArrayList<Obj>();
                for(Obj o : objs.values())
                    objsList.add(o);
                synchronized (armLock) {
                    armInterpreter.updateWorld(objsList);
                }

                TimeUtil.sleep(1000/30);
            }
        }
    }
}
