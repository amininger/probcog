package probcog.servo;

import java.util.HashMap;
import java.io.IOException;

import lcm.lcm.*;
import orc.*;
import april.util.TimeUtil;
import april.config.Config;
import april.jmat.MathUtil;

import probcog.lcmtypes.*;

public class AX12Controller extends Thread
{
    static final double DEFAULT_SPEED = 0.05;
    static final double DEFAULT_TORQUE = 0.6;
    static final double DEFAULT_ZERO = 150;
    static final double DEFAULT_ANGLE_THRESHOLD = 3.0;
    static final double ACTUAL_MINIMUM = 0.0;
    static final double ACTUAL_MAXIMUM = 300.0;
    static final int WAIT_DELAY_TIME = 100;
    static final int DEFAULT_UPDATE_PERIOD = 100;
    static final int MIN_PUBLISH_PERIOD = 10;

    static final int PAUSE_ORC_ESTOP = 987654321;

    public static final double SERVO_SPEED_MULTIPLIER = (2*Math.PI*114)/60.0;

    private AX12Servo servo;
    Orc orc;

    private int inverted;      // used when angle is backwards (rather than boolean)
    private double dfltSpeed;
    private double desiredAngle, desiredSpeed, desiredTorque;

    private double zeroAngle, minAngle, maxAngle;
    private int publishPeriod_ms;
    private String channel;

    private HashMap<Integer, Boolean> pauseSet;  // where key is unique ID and value is special position

    private StatusThread sThread;

    public boolean verbose;

    /**
     *  Constructor using all available defaults
     **/
    public AX12Controller(Orc _orc, String _channel, int _id)
    {
        this(_orc, _channel, _id, false,  DEFAULT_ZERO, ACTUAL_MINIMUM - DEFAULT_ZERO,
             ACTUAL_MAXIMUM - DEFAULT_ZERO, DEFAULT_SPEED, DEFAULT_UPDATE_PERIOD,
             false);
    }

    /**
     *  Constructor using config file straight
     **/
    public AX12Controller(Orc _orc, Config config)
    {
        this(_orc,
             config.requireString("publish_channel"),
             config.requireInt("servo_num"),
             config.requireBoolean("servo_angle_inverted"),
             config.requireDouble("servo_angle_set_zero"),
             config.requireDouble("servo_min_angle_degrees"),
             config.requireDouble("servo_max_angle_degrees"),
             config.requireDouble("servo_speed"),
             config.requireInt("servo_status_period_ms"),
             false);
    }

    /**
     *  Constructor with all parameters specified
     **/
    public AX12Controller(Orc _orc, String _channel, int _id, boolean _invertAngle,
                          double _zeroAngle, double _minAngle, double _maxAngle,
                          double speed, int _publishPeriod_ms)
    {
        this(_orc, _channel, _id, _invertAngle,  _zeroAngle, _minAngle, _maxAngle,
             speed, _publishPeriod_ms, false);
    }

    /**
     *  Constructor with all parameters specified and verbose
     **/
    public AX12Controller(Orc _orc, String _channel, int _id, boolean _invertAngle,
                          double _zeroAngle,double _minAngle, double _maxAngle,
                          double speed, int _publishPeriod_ms, boolean _verbose)
    {
        this.orc = _orc;

        // Boundary angles in degrees
        zeroAngle = _zeroAngle;
        minAngle = _minAngle;
        maxAngle = _maxAngle;
        verbose = _verbose;
        channel = _channel;
        publishPeriod_ms = _publishPeriod_ms;

        assert((zeroAngle >=0) && (zeroAngle <= 300));
        assert(((minAngle + zeroAngle) >=0) && ((minAngle + zeroAngle) <= 300));
        assert(((maxAngle + zeroAngle) >=0) && ((maxAngle + zeroAngle) <= 300));
        assert(maxAngle > minAngle);

        if (publishPeriod_ms < MIN_PUBLISH_PERIOD)
            publishPeriod_ms = MIN_PUBLISH_PERIOD;

        inverted = (_invertAngle ? -1 : 1);     // NOT INVERTED = 1
        initialize(_id, speed);
    }

    /**
     * Initialization for class.
     **/
    public void initialize(int id, double speed)
    {
        final int NUM_INIT_RETRIES = 10;
        desiredSpeed = speed;
        dfltSpeed = speed;
        desiredTorque = DEFAULT_TORQUE;

        pauseSet = new HashMap<Integer, Boolean>();

        servo = new AX12Servo(orc, id);
        boolean initialized = false;

        // insure there has been a valid ping before continuing
        for (int i = 0; i < NUM_INIT_RETRIES && !initialized; i++)
            initialized = servo.ping();
        if (!initialized)
        {
            System.out.println("ERR: Could not initialize Servo");
            System.exit(-1);
        }
        sThread = new StatusThread(LCM.getSingleton());
        sThread.start();
    }

    /**
     * Accessor function for maxAngle
     *
     * @return the set maxAngle
     **/
    public double getMaxAngle()
    {
        return maxAngle;
    }

    /**
     * Accessor function for minAngle
     *
     * @return the set minAngle
     **/
    public double getMinAngle()
    {
        return minAngle;
    }

    /**
     * The main servo controlling thread that sends the desired goal
     * for the servo only when one of the 3 goal parameters has
     * changed. These parameters are: angle, torque, and speed.
     **/
    public void run()
    {
        double newAngle=0, newSpeed=0, newTorque=0;
        long lastEstopCheck = 0;
        boolean estop  = false;
        boolean estopCur = false;
        boolean paused = false;
        boolean pausedCur = false;
        double angle = 0;
        long lastUpdate = 0;

        while(true)
        {
            long utime = TimeUtil.utime();
            boolean needUpdate = false;
            boolean checkEstop = ((utime - lastEstopCheck) > 50000);  // 1/20 sec

            // periodically check the HW estop state
            if (checkEstop) {
                lastEstopCheck = utime;
                estopCur = orc.getStatus().getEstopState();
            }
            synchronized(pauseSet){
                // check estop state
                //    - if just estopped,  set pos = SPECIAL
                //    - if just UNestopped remove pause (on estop)
                // check paused state (incl estop)
                //    - if just unpaused set needUpdate = true;
                //    - if just paused set pos = current;
                // else set pos = desired

                // update pauseSet with estop state (on toggle only)
                if (checkEstop && estopCur != estop) {
                    if (estopCur) {
                        pauseSet.put(PAUSE_ORC_ESTOP, false);  // false = goto special place
                        //angle = 0;
                    } else if (estop)
                        pauseSet.remove(PAUSE_ORC_ESTOP);
                        if (pauseSet.isEmpty())
                            pauseSet.notifyAll();
                }
                boolean curPaused = !pauseSet.isEmpty();

                // if just paused, lock angle to be current position (including estopped)
                if (curPaused && !paused) {
                    if (pauseSet.containsValue(false))
                        angle = 0;
                    else
                        angle = getServoAngle_deg(sThread.getCurrentStatus(),
                                                  TimeUtil.utime(), true);
                }
                else if (!curPaused && paused)
                    needUpdate = true;
                paused = curPaused;
                estop = estopCur;
                if (paused) {
                    try{
                        servo.setGoalDegrees(inverted*angle + zeroAngle,
                                             1.0, DEFAULT_TORQUE);
                        pauseSet.wait(100);
                    }
                    catch (InterruptedException ex){
                    }
                    continue;
                }
            }
            // the following line is required in cases such as (actual) AX12 reset
            needUpdate |= ((utime - lastUpdate) > 2E6);
            synchronized(this){
                needUpdate |= (newAngle != desiredAngle) ||
                              (newSpeed != desiredSpeed) ||
                              (newTorque != desiredTorque);
                newAngle = desiredAngle;
                newSpeed = desiredSpeed;
                newTorque = desiredTorque;
            }
            if (needUpdate)
            {
                lastUpdate = utime;
                if (verbose)
                    System.out.println("NFO: AX12 Controller changing goal on servo");
                servo.setGoalDegrees(newAngle + zeroAngle, newSpeed, newTorque);
            }
            TimeUtil.sleep(200);
        }
    }

    /**
     * Cause the servo to pause and stay in current location
     * @param pauseID an unique id that must be unpaused (same id)
     **/
    public void pause(int pauseID)
    {
        pause(pauseID, true);
    }

    /**
     * Cause the servo to pause
     * @param pauseID an unique id that must be unpaused (same id)
     * @param keepCurrentPosition,otherwise go to special location estop position
     **/
    public void pause(int pauseID, boolean keepCurrentPosition)
    {
        synchronized(pauseSet){
            pauseSet.put(pauseID, keepCurrentPosition);
        }
    }

    /**
     * Cause the servo to pause and stay in current location
     * @param pauseID an unique id that has been sent to pause(same id)
     **/
    public void unpause(int pauseID)
    {
        synchronized(pauseSet){
            pauseSet.remove(pauseID);
            if (pauseSet.isEmpty())
                pauseSet.notifyAll();
        }
    }

    /**
     * Sets the angle that the run method uses to control
     * the servo position actual angle sent to servo.
     *
     * @param newAngle in [minAngle, maxAngle]. But also within
     * [-ZERO, 300-ZERO].
    **/
    public void setAngleDeg(double newAngle)
    {
        if (newAngle < minAngle)
            newAngle = minAngle;
        if (newAngle > maxAngle)
            newAngle = maxAngle;

        newAngle *= inverted;
        synchronized(this){
            desiredAngle = newAngle;
        }
    }


    /**
     * Sets the angle that the run method uses to control
     * the servo position actual angle sent to servo.
     *
     * @param newAngle in radian version of [minAngle, maxAngle]. But
     * also within [-ZERO, 300-ZERO].
    **/
    public void setAngleRad(double newAngle)
    {
        newAngle = Math.toRadians(newAngle);
        setAngleDeg(newAngle);
    }

    /**
     * Sets the desired speed of the servo
     * @param newSpeed in [0,1]
     **/
    public void setSpeed(double newSpeed)
    {
        synchronized (this) {
            desiredSpeed = Math.min(Math.max(newSpeed, 0.001), 1.0);
        }
    }

    /**
     * Sets the desired torque of the servo
     *
     * @param newTorque in [0,1]
     **/
    public void setTorque(double newTorque)
    {
        if ((newTorque >= 0.0) && (newTorque <= 1.0))
            synchronized(this){
                desiredTorque = newTorque;
            }
    }

    /**
     * Sets the desired torque of the servo
     **/
    public void resetAngle()
    {
        setAngleDeg(zeroAngle);
    }

    /**
     * Sets the desired torque of the servo
     **/
    public void resetSpeed()
    {
        synchronized(this){
            desiredSpeed = dfltSpeed;
        }
    }

    /**
     * Sets the desired torque of the servo
     **/
    public void resetTorque()
    {
        synchronized(this){
            desiredTorque = DEFAULT_TORQUE;
        }
    }

    /**
     * Blocking call to wait for the servo to move to the anlge set
     * with one of the setAngle() calls.  This allows the calling
     * program to ability to block until the servo is within the
     * default threshold angle of the goal
     **/
    public boolean waitForGoal()
    {
        return waitForGoal(DEFAULT_ANGLE_THRESHOLD, Long.MAX_VALUE/1000);
    }

    /**
     * Blocking call to wait for the servo to move to the anlge set
     * with one of the setAngle() calls.  This allows the calling
     * program to ability to block until the servo is within some
     * specified angle of the goal
     *
     * @param threshold must be (0,300]
     * @param timeout_ms the max time to stay in this function
     * @return boolean for if goal reached by timeout_ms
     **/
    public boolean waitForGoal(double threshold, long timeout_ms)
    {
        double waitAngle = 0.0, angle;

        if (threshold <= 0)
            threshold = DEFAULT_ANGLE_THRESHOLD;
        long startTime = TimeUtil.utime();

        while((TimeUtil.utime() - startTime) < 1000*timeout_ms) {
            synchronized(pauseSet){
                while (!pauseSet.isEmpty()) {
                    try{
                        pauseSet.wait(WAIT_DELAY_TIME);
                    }
                    catch (InterruptedException ex){
                    }
                    continue;  // force timeout check
                }
            }

            angle = getServoAngle_deg(sThread.getCurrentStatus(), TimeUtil.utime(), false);
            angle *= inverted;

            synchronized(this){
                waitAngle = desiredAngle;
            }
            if (Math.abs(mod360(angle - waitAngle)) < threshold)
                    return true;
            TimeUtil.sleep(WAIT_DELAY_TIME);
        }
        return false;
    }

    /**
     * Degree version of MathUtil.mod2pi() to bound angles in [-180, 180]
     */
    public double mod360(double angle)
    {
        while(angle > 180)
            angle -= 360;
        while(angle < -180)
            angle += 360;
        return angle;
    }


    /**
     * Blocks forever till the servo's speed becomes zero
     **/
    public boolean waitForIdle()
    {
        return waitForIdle(Long.MAX_VALUE/1000);
    }

    /**
     * Blocks till the servo's speed becomes zero
     * @param timeout_ms the max time to stay in this function
     * @return boolean for if idle by timeout_ms
     */
    public boolean waitForIdle(long timeout_ms)
    {
        long startTime = TimeUtil.utime();

        while((TimeUtil.utime() - startTime) < 1000*timeout_ms)
        {
            synchronized(pauseSet){
                while (!pauseSet.isEmpty()) {
                    try{
                        pauseSet.wait(WAIT_DELAY_TIME);
                    }
                    catch (InterruptedException ex){
                    }
                    continue;  // force timeout check
                }
            }
            if (sThread.getCurrentStatus().speed == 0)
                    return true;
            TimeUtil.sleep(WAIT_DELAY_TIME);
        }
        return false;
    }

    /**
     * Blocks till the servo reaches goal and speed becomes zero
     */
    public boolean waitForGoalAndIdle()
    {
        return (waitForGoal() && waitForIdle());

    }

    public boolean waitForGoalAndIdle(long timeout_ms)
    {
        long utime = TimeUtil.utime();
        boolean b = waitForGoal(DEFAULT_ANGLE_THRESHOLD, timeout_ms);
        return (b && waitForIdle(timeout_ms - (TimeUtil.utime() - utime)/1000));
    }

    public boolean peakIdle()
    {
        return sThread.getCurrentStatus().speed == 0;
    }

    public boolean peakGoal()
    {
        return peakGoal(DEFAULT_ANGLE_THRESHOLD);
    }

    public boolean peakGoal(double threshold_deg)
    {
        double curAngle = inverted * getServoAngle_deg(sThread.getCurrentStatus(), TimeUtil.utime(), false);

        double waitAngle = desiredAngle;
        return (Math.abs(mod360(curAngle - waitAngle)) < threshold_deg);
    }

    /**
     * Static method to get the current angle of the servo in degrees from a status message
     *
     * @param status   lcm status message from servo
     * @param utime   the utime used when interpolating
     * @param interpolate  return angle in message or interpolate based on speed
     * @return angle in degrees
     **/
    public static double getServoAngle_deg(ax12_status_t status, long utime, boolean interpolate)
    {
        double angle_rad = getServoAngle_rad(status, utime, interpolate);
        return Math.toDegrees(angle_rad);
    }

    /**
     * Static method to get the current angle of the servo in degrees from a status message
     *
     * @param status   lcm status message from servo
     * @param utime   the utime used when interpolating
     * @param interpolate  return angle in message or interpolate based on speed
     * @return angle in radians
     **/
    public static double getServoAngle_rad(ax12_status_t status, long utime, boolean interpolate)
    {
        double angle = 0;

        if (status != null)
        {
            angle = status.position_radians;
            if (interpolate)
            {
                double speed = status.speed;
                double dt = (utime - status.utime) / 1000000.0;
                if (speed >= 1)
                    speed = (1 - speed);

                angle += speed * dt * SERVO_SPEED_MULTIPLIER;
            }
        } else
            System.out.println("WRN: No angle data in null servo status");
        return angle;
    }

    public double getCurrentAngle_deg()
    {
        return getServoAngle_deg(sThread.status, 0, false);
    }

    public double getCurrentAngle_rad()
    {
        return getServoAngle_rad(sThread.status, 0, false);
    }

    public boolean isAtAngle_deg(double deg)
    {
        return Math.abs(getCurrentAngle_deg()-deg) < DEFAULT_ANGLE_THRESHOLD;
    }

    public double getCurrentSpeed()
    {
        return sThread.status.speed;
    }

    /**
     * StatusThread periodically reads the status from the servo.
     * This status may be published over lcm but will always be
     * instantaneously available to the calling class for polling.  If
     * channel is null then the status is not published, otherwise it
     * will be publish on channel.
     **/
    public class StatusThread extends Thread
    {
        private ax12_status_t status;

        public boolean verbose;
        LCM lcm;

        StatusThread(LCM _lcm)
        {
            lcm = _lcm;
            status = new ax12_status_t();
            verbose = true;
        }

    /**
     * Accessor function to get most recent status from servo
     *
     * @return most recent status in a ax12-status_t variable
     **/
        public synchronized ax12_status_t getCurrentStatus()
        {
            return status;
        }

    /**
     * Periodically reads status from servo and stores this locally
     * and publishes that status if channel != null
     **/
        public void run()
        {
            while(true)
            {
                TimeUtil.sleep(publishPeriod_ms);
                ax12_status_t stat = readAX12Status();
                if (stat == null){
                    System.out.println("WRN: Null return from readAX12Status()");
                    continue;
                }
                synchronized(this){
                    status = stat;
                }
                // if channel is null then do not publish, this is how
                // the calling class can prevent from sending messages
                // over lcm
                if (channel != null)
                    lcm.publish(channel, stat);
            }
        }

    /**
     * Reads the servo and pacakges the data into a ax12_status_t
     *
     * @return the ax12-status_t variable
     **/
        private ax12_status_t readAX12Status()
        {
            ax12_status_t newStatus = null;
            AX12Status st = null;

            newStatus = new ax12_status_t();
            st = servo.getStatus();
            if (st == null)
                return null;

            newStatus.utime = st.utimeHost;
            newStatus.position_radians = Math.toRadians(inverted*(st.positionDegrees - zeroAngle));
            newStatus.speed = st.speed;
            newStatus.load = st.load;
            newStatus.voltage = st.voltage;
            newStatus.temperature = st.temperature;
            newStatus.error_flags = st.error_flags;
            return newStatus;
        }
    }

    public static void main(String args[])
    {
        int servoNum = 3;
        Orc orc = Orc.makeOrc();
        AX12Controller cntr = new AX12Controller(orc, "SERVO_STATUS", servoNum);
        cntr.run();
    }
}
