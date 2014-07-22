package probcog.servo;

import java.io.IOException;
import java.io.*;

import orc.*;

import april.util.TimeUtil;
import april.util.*;

import probcog.servo.*;
import probcog.util.*;

/** A class which controls a dynamixel to move a Hokuyo LIDAR. **/
public class Hokuyo3D implements Runnable
{
    static final double DFLT_TORQUE = 1.0;
    static final String DEVICE = "servo_hokuyo.";
    static final int BTN_SCAN_REQUESTED = 0xC0;

    // State variables for servo loop
    int desiredState;
    static final int STATE_DEFAULT = 0, STATE_DOUBLE_SCAN = 2;
    //STATE_SINGLE_SCAN = 1, STATE_DOUBLE_SCAN = 2, STATE_TRACKING = 3;

    private DynamixelController servoHok;
    private double minAngle, maxAngle, defaultAngle, trackingAngle;

    Object stateLock;
    long lastScanStart;

    boolean continuousScan;

    // Pause servo variables
    boolean paused = false;  // use for CMDS Frozen mode

    long cmdsUtime; // needed to properly handle CMDS message
    boolean ensureMonotonicMsgs;  // only accept newer msge (no loops in log playback)

    public boolean verbose;

    private boolean scanInProgress;

    public Hokuyo3D()
    {
        this(false);
    }

    public Hokuyo3D(boolean _verbose)
    {
        ensureMonotonicMsgs = Util.getConfig().getBoolean("lcm.accept_only_monotonic_utimes", true);

        double servoSetZero = Util.getConfig().requireInt(
            DEVICE + "servo_angle_set_zero");
        minAngle = Util.getConfig().requireDouble(
            DEVICE + "servo_min_angle_degrees");
        maxAngle = Util.getConfig().requireDouble(
            DEVICE + "servo_max_angle_degrees");
        defaultAngle = Util.getConfig().requireDouble(
            DEVICE + "servo_default_angle_degrees");
        trackingAngle = Util.getConfig().requireInt(
            DEVICE + "servo_tracking_angle_degrees");
        double speedHok = Util.getConfig().requireDouble(
            DEVICE + "servo_speed");
        int statusPeriod_ms = Util.getConfig().requireInt(
            DEVICE + "servo_status_period_ms");
        int id = Util.getConfig().requireInt(DEVICE + "servo_num");
        boolean inverted = Util.getConfig().requireBoolean(
            DEVICE + "servo_angle_inverted");
        String orcIpWithServos = Util.getConfig().requireString(
            "robot.orcHinge");

        continuousScan = Util.getConfig().requireBoolean(
            "HOKUYO_LIDAR.continuous_scan");

        assert (continuousScan);    // This is all we support right now

        verbose = _verbose;
        lastScanStart = -1;
        stateLock = new Object();

        Orc orc = Orc.makeOrc(orcIpWithServos);

        servoHok = new DynamixelController(orc, "DYNAMIXEL_STATUS_HOKUYO",
                                      id, inverted, servoSetZero, minAngle, maxAngle,
                                      speedHok, statusPeriod_ms, _verbose);
        servoHok.setTorque(DFLT_TORQUE);
        servoHok.start();
    }

    public void run()
    {
        long curTime = 0;

        int currentState = desiredState;

        goToAngle(defaultAngle);
        while(true)
        {
            // Ensure we notice all state changes which happen either while wait()
            // or during the execution of the switch
            synchronized(stateLock) {
                scanInProgress = false;

                if (!continuousScan && currentState == desiredState) {
                    try{
                        stateLock.wait();
                    }catch (InterruptedException ex){}
                }

                // Grab the current task
                currentState = desiredState;

                if (continuousScan)
                    desiredState = STATE_DOUBLE_SCAN;
                else
                    desiredState = STATE_DEFAULT;

                scanInProgress = true; //Blocks the gamepad from overwriting the next state
            }

            switch (currentState) {
                case STATE_DEFAULT:
                    goToAngle(defaultAngle);
                    //VelReqMonitor.sendVelRequest("HOKUYO3D", 0, 1.0);
                    break;
                case STATE_DOUBLE_SCAN: // For continuous scanning, loop this state
                    goToAngle(minAngle);
                    curTime = TimeUtil.utime();

                    // Scan Up
                    goToAngle(maxAngle);

                    // Scan down
                    goToAngle(minAngle);
                    break;
            }

        }
    }

    private void goToAngle(double angle)
    {
        final int ANGLE_THRESH = 5;

        servoHok.setAngleDeg(angle);
        while (!servoHok.waitForGoal(ANGLE_THRESH, 2000))
            ;
    }

    public synchronized void pauseServo()
    {
        if (!paused) { // save a synchronization call
            servoHok.pause(22);
            if (verbose)
                System.out.println("DBG: Pause");
        }
        paused = true;
    }

    public synchronized void unPauseServo()
    {
        if (paused) {  // save a synchronization call
            servoHok.unpause(22);
            if (verbose)
                System.out.println("DBG: Unpause");
        }
        paused = false;
    }

    public static void main(String args[])
    {
        new Hokuyo3D().run();
    }
}
