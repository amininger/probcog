package probcog.util;

import java.awt.image.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.lcmtypes.*;
import april.tag.*;
import april.util.*;

import probcog.camera.*;
import probcog.lcmtypes.*;
import probcog.servo.*;

public class TagDetectionGenerator
{
    // image queue
    final BlockingSingleQueue<CameraDriver.LazyImage> imageQueue = new BlockingSingleQueue<CameraDriver.LazyImage>();

    long lastDetection_ut;

    Config config;

    LCM lcm = LCM.getSingleton();

    boolean eStopped = true;

    TagFamily tf;
    TagDetector detector;

    boolean verbose = false;

    // detection parameter defaults
    double  segSigma            = 0.8;
    double  sigma               = 0;
    double  minMag              = 0.004;
    double  maxEdgeCost_deg     = 30;
    double  magThresh           = 1200;
    double  thetaThresh         = 100;
    int     errorBits           = 1;
    int     weightScale         = 100;
    boolean decimate            = false;

    // extrinsic camera parameters
    double poseToPan[][];
    double panToTilt[][];
    double tiltToCam[][];

    PanTiltServoController servos;
    AX12Tracker panTracker;
    AX12Tracker tiltTracker;

    public TagDetectionGenerator(PanTiltServoController _servos)
    {
        this.config = Util.getConfig();

        panTracker  = new AX12Tracker(config.requireString("servo_camera_pan.publish_channel"));
        tiltTracker = new AX12Tracker(config.requireString("servo_camera_tilt.publish_channel"));

        TagFamily tf = getTagFamily(config);
        this.detector = setupDetector(tf);

        servos = _servos;

        setupConfigConstants();

        new ProcessingThread().start();
    }

    public void addImage(CameraDriver.LazyImage lim)
    {
        if (lim.utime != lastDetection_ut){
            imageQueue.put(lim);
            lastDetection_ut = lim.utime;
        }
    }

    // Call for processing as images arrive
    private class ProcessingThread extends Thread
    {
        public void run()
        {
            while (true) {
                // Get newest image from queue and process
                CameraDriver.LazyImage lim = imageQueue.get();
                if (verbose)
                    System.out.println("DBG: TAG: Got image from queue");
                processImage(lim);
            }
        }
    }

    public TagFamily getTagFamily(Config config)
    {
        TagFamily tf = null;
        String family = config.requireString("tag_detection.tag.family");

        if (family.equals("Tag36h11")) {
            tf = new Tag36h11();
        } else if (family.equals("Tag16h5")) {
            tf = new Tag16h5();
        } else if (family.equals("Tag25h9")) {
            tf = new Tag25h9();
        } else {
            System.err.printf("ERR: TAG: Tag family '%s' not supported. Exiting.", family);
            System.exit(-1);
        }

        return tf;
    }

    public void setupConfigConstants()
    {
        verbose         = config.getBoolean("tag_detection.verbose",           verbose);
        segSigma        = config.getDouble("tag_detection.tag.segSigma",       segSigma);
        sigma           = config.getDouble("tag_detection.tag.sigma",          sigma);
        minMag          = config.getDouble("tag_detection.tag.minMag",         minMag);
        maxEdgeCost_deg = config.getDouble("tag_detection.tag.maxEdgeCost_deg",maxEdgeCost_deg);
        magThresh       = config.getDouble("tag_detection.tag.magThresh",      magThresh);
        thetaThresh     = config.getDouble("tag_detection.tag.thetaThresh",    thetaThresh);
        errorBits       = config.getInt("tag_detection.tag.errorBits",         errorBits);
        weightScale     = config.getInt("tag_detection.tag.weightScale",       weightScale);
        decimate        = config.getBoolean("tag_detection.tag.decimate",      decimate);

        // EXTRINSICS
        poseToPan       = ConfigUtil.getRigidBodyTransform(config,
                                                           "cameraCalibration.extrinsics.poseToPan");
        panToTilt       = ConfigUtil.getRigidBodyTransform(config,
                                                           "cameraCalibration.extrinsics.panToTilt");
        tiltToCam       = ConfigUtil.getRigidBodyTransform(config,
                                                           "cameraCalibration.extrinsics.tiltToCam");
    }

    public TagDetector setupDetector(TagFamily tf)
    {
        this.tf = tf;

        TagDetector d = new TagDetector(this.tf);
        this.tf.setErrorRecoveryBits(errorBits);

        d.sigma          = sigma;
        d.segSigma       = segSigma;
        d.segDecimate    = decimate;
        d.minMag         = minMag;
        d.maxEdgeCost    = Math.toRadians(maxEdgeCost_deg);
        d.magThresh      = magThresh;
        d.thetaThresh    = thetaThresh;
        d.WEIGHT_SCALE   = weightScale;

        return d;
    }

    public void processImage(CameraDriver.LazyImage lim)
    {
        ArrayList<TagDetection> detections;

        ax12_status_t pan  = panTracker.get(lim.utime);
        ax12_status_t tilt = tiltTracker.get(lim.utime);
        if (pan == null || tilt == null)
            return;

        double panAngle_rad  = pan.position_radians;
        double tiltAngle_rad = tilt.position_radians;

        BufferedImage im = lim.getImage();
        detections = detector.process(im, new double[] {im.getWidth()/2.0,
                                                        im.getHeight()/2.0});

        if (detections.size() > 0) {
            System.out.println("detections found: "+detections);

            pan_tilt_tag_detections_t tagList = new pan_tilt_tag_detections_t();
            tagList.utime = lim.utime;

            // camera-to-pose transformation
            tagList.cam_to_pose = LinAlg.multiplyMany(poseToPan,
                                              LinAlg.rotateZ(panAngle_rad),
                                              panToTilt,
                                              LinAlg.rotateZ(tiltAngle_rad),
                                              tiltToCam);

            tagList.ndetects = detections.size();
            tagList.detections = new tag_detection_t[tagList.ndetects];
            for (int i = 0; i < detections.size(); i++) {
                TagDetection d = detections.get(i);
                tag_detection_t td = new tag_detection_t();
                tagList.detections[i] = td;
                td.id = d.id;
                td.errors = d.hammingDistance;
                td.homography = d.homography;
                td.hxy = d.hxy;
            }
            lcm.publish("TAG_DETECTIONS", tagList);
        }
    }
}
