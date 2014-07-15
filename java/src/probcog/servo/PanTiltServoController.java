package probcog.servo;

import java.io.IOException;

import orc.Orc;
import lcm.lcm.*;

import april.util.TimeUtil;
import april.config.ConfigUtil;
import april.jmat.LinAlg;
import april.lcmtypes.pose_t;

import probcog.servo.DynamixelController;
import probcog.util.Util;
import probcog.lcmtypes.*;

/** This class allows multiple threads to utilyze the pan/tilt servos
 *  without wasting memory and acts as central controller of the servos
 *  so that there are no race conditions based on asynchronous LCM events.
 **/
public class PanTiltServoController
{
    // do not give access to these servos
    private DynamixelController panServo;
    private DynamixelController tiltServo;

    private int lastServoPriority = Integer.MAX_VALUE;

    // utime expiration of last command
    private long servoExpiration_ut;

    // tracking degrees vs 3D point
    private double [] trackPoint = null;  // null mean do not track
    private TrackingThread tThread;
    private int angleOffset;              // angle offset to force tracking thread to search

    // speeds
    private double tiltSpeed;
    private double panSpeed;

    static double minPanAngle = Util.getConfig().getChild("servo_camera_pan").requireDouble("servo_min_angle_degrees");
    static double maxPanAngle = Util.getConfig().getChild("servo_camera_pan").requireDouble("servo_max_angle_degrees");

    static double minTiltAngle = Util.getConfig().getChild("servo_camera_tilt").requireDouble("servo_min_angle_degrees");
    static double maxTiltAngle = Util.getConfig().getChild("servo_camera_tilt").requireDouble("servo_max_angle_degrees");

    double camToRobot[][]; // only calculate once;

    static boolean verbose = false;


    public PanTiltServoController()
    {
        Orc orc = Orc.makeOrc();

        panServo = new DynamixelController(orc, Util.getConfig().getChild("servo_camera_pan"));
        tiltServo = new DynamixelController(orc, Util.getConfig().getChild("servo_camera_tilt"));

        // extrinsic camera parameters
        double poseToPan[][] = ConfigUtil.getRigidBodyTransform(Util.getConfig(),
                                                                "cameraCalibration.extrinsics.poseToPan");
        double panToTilt[][] = ConfigUtil.getRigidBodyTransform(Util.getConfig(),
                                                                "cameraCalibration.extrinsics.panToTilt");
        double tiltToCam[][] = ConfigUtil.getRigidBodyTransform(Util.getConfig(),
                                                                "cameraCalibration.extrinsics.tiltToCam");

        // rotations = 0, so don't bother with identity rotation matrices
        camToRobot  = LinAlg.multiplyMany(poseToPan, panToTilt, tiltToCam);

        verbose |= Util.getConfig().getBoolean("camera_driver.pan_tilt_servos.verbose", false);

        tThread = new TrackingThread();
        if (Util.getConfig().getBoolean("camera_driver.pan_tilt_servos.scan", false)) {
            new ScanningThread().start();
        }
        tThread.start();
        panServo.start();
        tiltServo.start();

        tiltServo.setAngleDeg(minTiltAngle);    // XXX Correct place for this?
    }

    /* Set Pan Servo speed and angle based on a 3D point with the
     * priority/timeout system.
     */
    public boolean setTrackPoint(double []localPoint, double pan_speed, double tilt_speed,
                                 int priority, long timeout_ms)
    {
        return setTrackPoint(localPoint, pan_speed, tilt_speed, priority, timeout_ms, 0);
    }

    /* Set Pan Servo speed and angle based on a 3D point with the
     * priority/timeout system. Allows user to enforce an angle offset
     */
    public synchronized boolean setTrackPoint(double []localPoint, double pan_speed, double tilt_speed,
                                              int priority, long timeout_ms, int angleAddition)
    {
        long utime = TimeUtil.utime();

        boolean valid = (priority <= lastServoPriority || utime > servoExpiration_ut);
        if (valid) {
            if (localPoint == null || localPoint.length < 2)
                return false;
            if (localPoint.length == 3)
                trackPoint = localPoint;
            else
                trackPoint = new double[]{localPoint[0], localPoint[1], 0};
            servoExpiration_ut = utime + 1000*timeout_ms;
            if (priority != lastServoPriority)
                System.out.println("NFO: SERVOS: PAN/TILT controlled with priority " + priority);
            lastServoPriority = priority;
            angleOffset = angleAddition;
            panSpeed = pan_speed;
            tiltSpeed = tilt_speed;
        }
        return valid;
    }

    /* Set Pan Servo speed and angle based on priority system
     */
    public synchronized boolean setPanSpeedAngle_deg(double speed, double angle_deg,
                                                     int priority, long timeout_ms)
    {
        long utime = TimeUtil.utime();

        boolean valid = (priority <= lastServoPriority || utime > servoExpiration_ut);
        if (valid) {
            panServo.setSpeed(speed);
            panServo.setAngleDeg(angle_deg);
            servoExpiration_ut = utime + 1000*timeout_ms;
            if (priority != lastServoPriority)
                System.out.println("NFO: SERVOS: PAN controlled with priority " + priority);
            lastServoPriority = priority;
            trackPoint = null;
            panSpeed = speed;
            angleOffset = 0;
        }
        return valid;
    }

    // sets tilt angle with priority and lasts timeout_ms long
    public synchronized boolean setTiltSpeedAngle_deg(double speed, double angle_deg, int priority, long timeout_ms)
    {
        long utime = TimeUtil.utime();

        boolean valid = (priority <= lastServoPriority || utime > servoExpiration_ut);
        if (valid) {
            tiltServo.setSpeed(speed);
            tiltServo.setAngleDeg(angle_deg);
            servoExpiration_ut = utime + 1000*timeout_ms;
            if (priority != lastServoPriority)
                System.out.println("NFO: SERVOS: TILT controlled with priority " + priority);
            lastServoPriority = priority;
            trackPoint = null;
            tiltSpeed = speed;
        }
        return valid;
    }

    public double getCurrentPanAngle_deg()
    {
        return panServo.getCurrentAngle_deg();
    }

    public double getCurrentTiltAngle_deg()
    {
        return tiltServo.getCurrentAngle_deg();
    }

    /** Peak to see if pan servo is at goal
     * @param priority of calling thread
     * @param threshold_deg for when to return true
     * @return -1 on bad priority, 0 on success, and 1 on not within thresh
     **/
    public synchronized int peakGoalPan(int priority, double threshold_deg)
    {
        if (priority != lastServoPriority)
            return -1;
        else
            return (panServo.peakGoal(threshold_deg) ? 0 : 1);
    }

    /** Peak to see if tilt servo is at goal
     * @param priority of calling thread
     * @param threshold_deg for when to return true
     * @return -1 on bad priority, 0 on success, and 1 on not within thresh
     **/
    public synchronized int peakGoalTilt(int priority, double threshold_deg)
    {
        if (priority != lastServoPriority)
            return -1;
        else
            return (tiltServo.peakGoal(threshold_deg) ? 0 : 1);
    }

    public boolean waitForPanGoal(int priority)
    {
        if (!panServo.waitForGoalAndIdle())
            return false;
        return (priority == lastServoPriority);
    }

    public boolean waitForPanGoal(int priority, long timeout_ms)
    {
        if (!panServo.waitForGoalAndIdle(timeout_ms))
            return false;
        return (priority == lastServoPriority);
    }

    public boolean waitForTiltGoal(int priority)
    {
        if (!tiltServo.waitForGoalAndIdle())
            return false;
        return (priority == lastServoPriority);
    }

    public boolean waitForTiltGoal(int priority, long timeout_ms)
    {
        if (!tiltServo.waitForGoalAndIdle(timeout_ms))
            return false;
        return (priority == lastServoPriority);
    }

    public double getMinPanAngle()
    {
        return minPanAngle;
    }

    public double getMaxPanAngle()
    {
        return maxPanAngle;
    }

    public double getMinTiltAngle()
    {
        return minTiltAngle;
    }

    public double getMaxTiltAngle()
    {
        return maxTiltAngle;
    }

    class ScanningThread extends Thread
    {

        final int PERIOD = 100;
        final double PAN_SPEED = Util.getConfig().getDouble("camera_driver.pan_scan_speed", 0.1);
        final double TILT_SPEED = 0.05;
        final int SCAN_PRIORITY = 99; // lowest of priorities
        final int TILT_ANGLE = 10;

        public void run()
        {

            // Scanning Angle Limits
            int maxPan  =  200;  // beyond limits, thus will use max
            int minPan  = -200;  // beyond limits, thus will use min

            // current angles
            int panAngle = minPan;

            while (true) {
                if (true) {
                    // This doesn't do anything...?
                    if (panServo.peakGoal(5)) {
                    }
                    do {
                        setPanSpeedAngle_deg(PAN_SPEED, panAngle,
                                             SCAN_PRIORITY, Long.MAX_VALUE);
                        setTiltSpeedAngle_deg(TILT_SPEED, TILT_ANGLE,
                                              SCAN_PRIORITY, Long.MAX_VALUE);
                    } while (!waitForPanGoal(SCAN_PRIORITY) ||
                             !waitForTiltGoal(SCAN_PRIORITY));
                    panAngle = (panAngle == minPan ? maxPan : minPan);
                }
                TimeUtil.sleep(PERIOD);
            }
        }
    }

    // Cause the robot's camera to look at the tag/OOI
    class TrackingThread extends Thread implements LCMSubscriber
    {
        final double HFOV = 80;
        final double aspect = 752.0 / 480.0 / 2;      // Aspect ratio
        final double VFOV = HFOV/aspect;

        private pose_t pose;

        private boolean paused;

        long cmdsUtime; // needed to properly handle CMDS message
        boolean ensureMonotonicMsgs;  // only accept newer msge (no loops in log playback)

        public TrackingThread()
        {
            ensureMonotonicMsgs = Util.getConfig().getBoolean(
                "lcm.accept_only_monotonic_utimes", true);
            LCM.getSingleton().subscribe("POSE", this);
        }



        public synchronized void pauseServos()
        {
            if (!paused) {  // save synchronization call
                tiltServo.pause(22);
                panServo.pause(22);
            }
            paused = true;
        }

        public synchronized void unPauseServos()
        {
            if (paused) {  // save synchronization call
                tiltServo.unpause(22);
                panServo.unpause(22);
            }
            paused = false;
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                messageReceivedEx(lcm, channel, ins);
            } catch (IOException ex) {
                System.out.println("WRN: SERVOS: "+ex);
            }
        }

        public void messageReceivedEx(LCM lcm, String channel,
                                      LCMDataInputStream ins) throws IOException
        {
            if (channel.equals("POSE")) {
                pose = new pose_t(ins);
            }
        }

        public void run()
        {
            final double ALPHA = 0.66;

            double curTrackPoint[] = null;
            int curPriority;
            double desiredPanAngle = panServo.getCurrentAngle_deg();
            double desiredTiltAngle = tiltServo.getCurrentAngle_deg();
            double curPanSpeed;
            double curTiltSpeed;

            System.out.println("NFO: SERVOS: Starting tracking thread");
            int offset = 0; // search angle

            while (true) {
                pose_t obsPose = pose;
                TimeUtil.sleep(50);

                double curPanAngle = panServo.getCurrentAngle_deg();
                double curTiltAngle = tiltServo.getCurrentAngle_deg();
                long utime = TimeUtil.utime();
                synchronized (PanTiltServoController.this) {
                    curTrackPoint = trackPoint;
                    curPriority = lastServoPriority;
                    curPanSpeed = panSpeed;
                    curTiltSpeed = tiltSpeed;
                    offset = angleOffset;
                    if (utime > servoExpiration_ut)
                        continue;
                }
                if (curTrackPoint == null || obsPose == null)
                    continue;
                if (verbose)
                    System.out.printf("DBG: SERVOS: Tracking (%3.2f,%3.2f,%3.2f)\n",
                                      curTrackPoint[0], curTrackPoint[1], curTrackPoint[2]);

                double Observer[][] = LinAlg.quatPosToMatrix(obsPose.orientation, obsPose.pos);
                double ObserverXYZRPY[] = LinAlg.matrixToXyzrpy(Observer);

                double curCamPose[][]  = LinAlg.multiplyMany(Observer, camToRobot);

                // camera roll/pitch/yaw are in camera axis (not what we want here, use robot rotations)
                double cameraXYZRPY[] = new double[]{curCamPose[0][3],
                                                     curCamPose[1][3],
                                                     curCamPose[2][3],
                                                     ObserverXYZRPY[3],
                                                     ObserverXYZRPY[4],
                                                     ObserverXYZRPY[5]};

                double goal[][] = LinAlg.translate(curTrackPoint);

                double toGoal[][] = LinAlg.multiplyMany(LinAlg.inverse(LinAlg.xyzrpyToMatrix(cameraXYZRPY)), goal);
                // [dX, dY, dZ] from camera (x,y,z) aligned to robot
                // frame (not camera's goofy frame)
                double toGoalXYZ[] = LinAlg.resize(LinAlg.matrixToXyzrpy(toGoal), 3);

                if (verbose) {
                    System.out.println("DBG: SERVOS: ");
                    LinAlg.print(toGoalXYZ);
                    System.out.println();
                }
                double dist = LinAlg.magnitude(toGoalXYZ);
                if (dist == 0)
                    continue;

                desiredPanAngle = Math.toDegrees(Math.atan2(toGoalXYZ[1], toGoalXYZ[0]));
                desiredTiltAngle = -Math.toDegrees(Math.asin(toGoalXYZ[2] / dist));

                if (verbose)
                    System.out.printf("\nDBG: SERVOS: (pan,tilt): CUR - Desired: " +
                                      "(%3.5f, %3.5f) (%3.5f, %3.5f)\n",
                                      curPanAngle, curTiltAngle,
                                      desiredPanAngle, desiredTiltAngle);

                // low-pass
                desiredPanAngle = (ALPHA) * desiredPanAngle + (1 - ALPHA) * curPanAngle;
                desiredTiltAngle = (ALPHA) * desiredTiltAngle + (1 - ALPHA) * curTiltAngle;
                synchronized (PanTiltServoController.this) {
                    if (trackPoint != null) {
                        panServo.setSpeed(curPanSpeed);
                        tiltServo.setSpeed(curTiltSpeed);
                        panServo.setAngleDeg(desiredPanAngle + offset);
                        tiltServo.setAngleDeg(desiredTiltAngle);
                    }
                }
            }
        }
    }
}
