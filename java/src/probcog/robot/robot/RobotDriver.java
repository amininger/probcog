package probcog.robot.robot;

import java.util.*;
import java.io.*;

import lcm.lcm.*;

import april.config.*;
import april.jmat.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.lcmtypes.*;

import probcog.robot.util.*;

import orc.*;
import java.net.*;

public class RobotDriver implements LCMSubscriber
{
    public final static int RIGHT_VERT_AXIS = 3;
    public final static int RIGHT_HORZ_AXIS = 2;
    public final static int LEFT_VERT_AXIS = 1;
    public final static int LEFT_HORZ_AXIS = 0;
    public final static int DPAD_VERT_AXIS = 5;
    public final static int DPAD_HORZ_AXIS = 4;

    public final static int LEFT = 0;
    public final static int RIGHT = 1;

    public final static int BTN_MANUAL_MASK  = 0x30;  // either of the top trigger btns
    public final static int MOTOR_PERIOD_MS = 20;

    //public static final double BASELINE_METERS = 0.3429;
    public static final double BASELINE_METERS = 0.46;

    public static final double DEADBAND_THRESH = 0.05; // consider velocities less than this to be zero.

    LCM lcm = LCM.getSingleton();

    //Config config = RobotUtil.getConfig();

    ExpiringMessageCache<gamepad_t> udppadCache = new ExpiringMessageCache<gamepad_t>(0.5);

    ExpiringMessageCache<diff_drive_t> diffDriveCache = new ExpiringMessageCache<diff_drive_t>(0.2);

    Orc orcLeft = Orc.makeOrc("192.168.237.8");//RobotUtil.getConfig().requireString("robot.orcLeft"));
    Orc orcRight = Orc.makeOrc("192.168.237.7");//RobotUtil.getConfig().requireString("robot.orcRight"));

    // FRONT=0, REAR=1 (always!)
    Motor motorFR = new Motor(orcRight, 0, false);
    Motor motorBR = new Motor(orcRight, 1, false);
    Motor motorFL = new Motor(orcLeft,  0, true);
    Motor motorBL = new Motor(orcLeft,  1, true);
    QuadratureEncoder encoderFR = new QuadratureEncoder(orcRight, 0, true);
    QuadratureEncoder encoderBR = new QuadratureEncoder(orcRight, 1, true);
    QuadratureEncoder encoderFL = new QuadratureEncoder(orcLeft,  0, false);
    QuadratureEncoder encoderBL = new QuadratureEncoder(orcLeft,  1, false);

    VelocityController vcFR = new VelocityController();
    VelocityController vcBR = new VelocityController();
    VelocityController vcFL = new VelocityController();
    VelocityController vcBL = new VelocityController();

    UDPDriveThread udt;

    public RobotDriver()
    {
        lcm.subscribe("DIFF_DRIVE", this);
        lcm.subscribe("GAMEPAD", this);

        MotorThread mt = new MotorThread();
        mt.start();

        EncoderThread et = new EncoderThread();
        et.start();

        UnfaultThread ut = new UnfaultThread();
        ut.start();

        udt = new UDPDriveThread();
        udt.start();

        UDPAdvertiseThread uat = new UDPAdvertiseThread();
        uat.start();
    }

    class MotorThread extends Thread
    {
        double lastSpeedLimit = 0;
        long lastSpeedLimitUtime = 0;

        /** Periodically recompute motor commands as a function of
         * DIFF_DRIVE, CMDS, GAMEPAD, VEL_REQUEST, etc.
         **/
        public void run()
        {
            double[] motorCmd = null;

            while (true) {
                TimeUtil.sleep(MOTOR_PERIOD_MS);

                long now = TimeUtil.utime();

                // default values: stop
                double cmdLR[] = new double[2];

                diff_drive_t msg = diffDriveCache.get();
                if (msg != null)
                    cmdLR = new double[] { msg.left, msg.right };

                gamepad_t gp = null;
                gamepad_t gp_udp = udppadCache.get();

                gp = gp_udp;
                String gpName = "GAMEPAD_UDP";

                boolean gpOverride = false;
                String cmdOrigin = "Default";

                // gamepad overrides all others
                if (gp != null && (gp.buttons & BTN_MANUAL_MASK) > 0) {
                    double speed = -gp.axes[RIGHT_VERT_AXIS];
                    double turn = gp.axes[RIGHT_HORZ_AXIS];

                    cmdOrigin = gpName;
                    cmdLR = new double[] { speed + turn, speed - turn };
                    normalizeLR(cmdLR);

                    gpOverride = true;
                }

                // limit turn rate
                if (!gpOverride) {
                    double turn = Math.abs(cmdLR[0]-cmdLR[1]);
                    double max_turn = 0.5;

                    if (turn > max_turn) {
                        cmdLR[0] *= max_turn / turn;
                        cmdLR[1] *= max_turn / turn;
                    }
                }

                // Now, send the motor commands.
                if (true) {
                    double dt = MOTOR_PERIOD_MS / 1000.0;

                    // even with only two encoders, this block is valid -- we take care
                    // of dealing with only two encoders later in the file
                    double pwmFL = vcFL.compute(dt, cmdLR[0]);
                    double pwmBL = vcBL.compute(dt, cmdLR[0]);
                    double pwmFR = vcFR.compute(dt, cmdLR[1]);
                    double pwmBR = vcBR.compute(dt, cmdLR[1]);

                    Motor.setMultiplePWM(new Motor[] { motorFL, motorBL },
                                         new double[] { pwmFL, pwmBL });
                    Motor.setMultiplePWM(new Motor[] { motorFR, motorBR },
                                         new double[] { pwmFR, pwmBR });

                    quad_drive_t quad = new quad_drive_t();
                    quad.utime = TimeUtil.utime();
                    quad.fl = pwmFL;
                    quad.bl = pwmBL;
                    quad.br = pwmBR;
                    quad.fr = pwmFR;
                    quad.cmdOrigin = cmdOrigin;
                    lcm.publish("ROBOT_QUAD_DRIVE", quad);
                }
            }
        }
    }

    static void normalizeLR(double cmdLR[])
    {
        double max = Math.max(Math.abs(cmdLR[0]), Math.abs(cmdLR[1]));
        if (max > 1) {
            cmdLR[0] /= max;
            cmdLR[1] /= max;
        }
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(lcm, channel, ins);
        } catch (IOException ex) {
            System.out.println("WRN: IOException "+channel+": "+ex);
        }
    }

    public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals("DIFF_DRIVE")) {
            diff_drive_t msg = new diff_drive_t(ins);
            diffDriveCache.put(msg, msg.utime);
        }
    }

    public class EncoderThread extends Thread
    {
        boolean lastEstop = true; // robot should warn if it's free to drive when it starts up.

        public void run()
        {
            while (true) {
                TimeUtil.sleep(10);

                OrcStatus statusLeft = orcLeft.getStatus();
                OrcStatus statusRight = orcRight.getStatus();

                if (true) {
                    motor_feedback_t msg = new motor_feedback_t();
                    msg.utime = TimeUtil.utime();
                    msg.nmotors = 4;
                    msg.encoders = new int[msg.nmotors];
                    msg.encoders[0] = encoderFL.getPosition(statusLeft);
                    msg.encoders[1] = encoderBL.getPosition(statusLeft);
                    msg.encoders[2] = encoderBR.getPosition(statusRight);
                    msg.encoders[3] = encoderFR.getPosition(statusRight);

                    msg.current = new double[msg.nmotors];
                    msg.current[0] = motorFL.getCurrentFiltered(statusLeft);
                    msg.current[1] = motorBL.getCurrentFiltered(statusLeft);
                    msg.current[2] = motorBR.getCurrentFiltered(statusRight);
                    msg.current[3] = motorFR.getCurrentFiltered(statusRight);

                    msg.applied_voltage = new double[msg.nmotors];
                    msg.applied_voltage[0] = motorFL.getPWM(statusLeft) * statusLeft.getBatteryVoltage();
                    msg.applied_voltage[1] = motorBL.getPWM(statusLeft) * statusLeft.getBatteryVoltage();
                    msg.applied_voltage[2] = motorBR.getPWM(statusRight) * statusRight.getBatteryVoltage();
                    msg.applied_voltage[3] = motorFR.getPWM(statusRight) * statusRight.getBatteryVoltage();

                    lcm.publish("MOTOR_FEEDBACK", msg);
                }

                if (true) {
                    boolean estop = statusLeft.getEstopState() || statusRight.getEstopState();

                    int encoderBLpos = encoderBL.getPosition(statusLeft);
                    int encoderBRpos = encoderBR.getPosition(statusRight);

                    double voltageFL = motorFL.getPWM(statusLeft) * statusLeft.getBatteryVoltage();
                    double voltageFR = motorFR.getPWM(statusRight) * statusRight.getBatteryVoltage();

                    double currentFL = motorFL.getCurrentFiltered(statusLeft);
                    double currentFR = motorFR.getCurrentFiltered(statusRight);

                    // update velocity controllers
                    vcFL.updateEncodersAndEstop(statusLeft.utimeOrc, voltageFL, currentFL, estop);
                    vcBL.updateEncodersAndEstop(statusLeft.utimeOrc, encoderBLpos, estop);
                    vcBR.updateEncodersAndEstop(statusRight.utimeOrc, encoderBRpos, estop);
                    vcFR.updateEncodersAndEstop(statusRight.utimeOrc, voltageFR, currentFR, estop);

                    if (estop && !lastEstop)
                        speak("E stop!");
                    if (!estop && lastEstop)
                        speak("E stop released.");
                    lastEstop = estop;
                }

//                  System.out.printf("%15f %15f %15f %15f\n", vcFL.velocity, vcFL.goal, vcFL.integral, vcFL.cmd);
//                  System.out.printf("%15f %15f %15f %15f\n", vcBL.velocity, vcBL.goal, vcBL.integral, vcBL.cmd);
            }
        }
    }

    class UDPAdvertiseThread extends Thread
    {
        public void run()
        {
            DatagramSocket sock;
            InetAddress broadcastAddr;

            try {
                sock = new DatagramSocket(35998, Inet4Address.getByName("192.168.3.6"));//+RobotUtil.getID()));
                sock.setBroadcast(true);
                broadcastAddr = Inet4Address.getByName("255.255.255.255");
            } catch (Exception ex) {
                System.out.println("ex: "+ex);
                return;
            }

            while (true) {

                try {
                    ByteArrayOutputStream bouts = new ByteArrayOutputStream();
                    DataOutputStream outs = new DataOutputStream(bouts);

                    outs.write('S');
                    outs.write('R');
                    outs.write('B');
                    outs.write('T');

                    outs.write(6);

                    OrcStatus statusRight = orcRight.getStatus();
                    outs.write(statusRight.getEstopState() ? 1 : 0);
                    outs.writeInt((int) (statusRight.getBatteryVoltage() * 100));
                    outs.write(udt.getNumSenders());

                    outs.flush();

                    byte b[] = bouts.toByteArray();
                    DatagramPacket packet = new DatagramPacket(b, b.length, broadcastAddr, 31157);

                    sock.send(packet);

                    TimeUtil.sleep(100);
                } catch (Exception ex) {
                    System.out.println("Exception: "+ex);
                    TimeUtil.sleep(1000);
                }
            }
        }
    }

    class UDPDriveThread extends Thread
    {
        HashMap<InetAddress, Long> senderTable = new HashMap<InetAddress, Long>();

        public int getNumSenders()
        {
            long now = TimeUtil.utime();
            int cnt = 0;

            for (long v : senderTable.values()) {
                double dt = (now - v) / 1.0E6;
                if (dt < 1.0)
                    cnt++;
            }

            return cnt;
        }

        public void run()
        {
            DatagramSocket sock;
            DatagramPacket packet = new DatagramPacket(new byte[1024], 1024);

            try {
                sock = new DatagramSocket(31156);
            } catch (SocketException ex) {
                System.out.println("Exception: "+ex);
                return;
            }

            while (true) {
                try {
                    sock.receive(packet);

                    if (packet.getLength() != 8)
                        continue;

                    byte data[] = packet.getData();
                    if (data[0] != 'D' || data[1] != 'R' || data[2] != 'B' || data[3] != 'T') {
                        continue;
                    }

                    int robotId = data[4]&0xff;

                    // not for us?
                    if (robotId != 6) {
                        continue;
                    }

                    senderTable.put(packet.getAddress(), TimeUtil.utime());

                    int fingerDown = data[5]&0xff;
                    double x = ((data[6]&0xff) - 127) / 127.0;
                    double y = ((data[7]&0xff) - 127) / 127.0;

                    gamepad_t gp = new gamepad_t();
                    gp.utime = TimeUtil.utime();
                    gp.present = true;
                    gp.naxes = 6;
                    gp.axes = new double[gp.naxes];
                    gp.axes[RIGHT_VERT_AXIS] = y;
                    gp.axes[RIGHT_HORZ_AXIS] = x;
                    gp.buttons = BTN_MANUAL_MASK;

                    udppadCache.put(gp, gp.utime);

                    lcm.publish("GAMEPAD_UDP", gp);
                } catch (IOException ex) {
                }
            }
        }
    }

    class UnfaultThread extends Thread
    {
        public void run()
        {
            while (true) {
                TimeUtil.sleep(2000);

                OrcStatus statusRight = orcRight.getStatus();
                OrcStatus statusLeft = orcLeft.getStatus();

                boolean faultFR = motorFR.isFault(statusRight);
                boolean faultBR = motorBR.isFault(statusRight);
                boolean faultBL = motorBL.isFault(statusLeft);
                boolean faultFL = motorFL.isFault(statusLeft);

                if (faultFR)
                    motorFR.clearFault();

                if (faultBR)
                    motorBR.clearFault();

                if (faultBL)
                    motorBL.clearFault();

                if (faultFL)
                    motorFL.clearFault();

                if (faultFR || faultBR || faultBL || faultFL)
                    System.out.println("WRN: Clearing motor fault");
            }
        }
    }

    class VelocityController
    {
        double velocity;
        double goal;

        long lastutime;
        int lastposition;

        double integral;

        double Ki = 4.0;

        double cmd;

        boolean estop;

        // convert ticks to normalized speed
        //static final double VELOCITY_SCALE = 1.0 / 2400.0; // full-resolution encoders
        static final double VELOCITY_SCALE = 1.0 / 1200.0; // half-resolution encoders
        static final double INTEGRAL_WINDUP_MAX = 0.5;

        public void updateEncodersAndEstop(long utime, int position, boolean estop)
        {
            this.estop = estop;
            double dt = (utime - lastutime) / 1000000.0;

            if (lastutime == 0 || Math.abs(dt) > 1.0) {
                // Resync: something goofy happened.
                System.out.println("NFO: Resetting velocity estimate");
                lastutime = utime;
                lastposition = position;
                return;
            }

            // only compute velocity after fairly large intervals of time.
            if (dt > 0.060) {
                velocity = (position - lastposition) / dt;
                velocity *= VELOCITY_SCALE;

                lastutime = utime;
                lastposition = position;
            }
        }

        // same as method above, but using current/voltage model
        public void updateEncodersAndEstop(long utime, double v, double c, boolean estop)
        {
            this.estop = estop;

            double p[] = new double[] { 2.7388, -187.7056, 1.9940, 15.1520 };
            double vel = v*p[0] + c*p[1] + v*v*p[2] + c*c*p[2];
            if (v < 0)
                vel = -vel;

            if (Math.abs(v) < 0.1)
                vel = 0;

            velocity = vel * VELOCITY_SCALE;
            lastutime = utime;
        }

        /**
         * @param goal;  // goal speed (normalized [-1, 1])
         **/
        public double compute(double dt, double goal)
        {
            this.goal = goal;
            double minpwm = 0.15; // ff pwm corresponding to speed = 0
            double maxpwm = 1.0;  // ff pwm corresponding to speed = 1

            // by commanding goal=0 when estopped, we prevent integrator windup.
            if (estop)
                goal = 0;

            double ffpwm = minpwm + (maxpwm - minpwm)*Math.abs(goal);
            if (goal < 0)
                ffpwm = -ffpwm;

            // dead band
            if (Math.abs(goal) < DEADBAND_THRESH)
                ffpwm = 0;

            integral += dt*(goal - velocity);
            if (integral > INTEGRAL_WINDUP_MAX)
                integral = INTEGRAL_WINDUP_MAX;
            if (integral < -INTEGRAL_WINDUP_MAX)
                integral = -INTEGRAL_WINDUP_MAX;

            cmd = ffpwm + Ki*integral;
            if (cmd > 1)
                cmd = 1;
            if (cmd < -1)
                cmd = -1;
            if (Math.abs(goal) < DEADBAND_THRESH) {
                cmd = 0;
                integral = 0;
            }

            return cmd;
        }
    }

    // unconditionally speak.
    void speak(String s)
    {
        long nowutime = TimeUtil.utime();

        // generate alarm.
        speak_t spk = new speak_t();
        spk.utime = nowutime;
        spk.voice = "english:p99:a200:s180";
        spk.priority = 2;
        spk.message = s;

        lcm.publish("SPEAK", spk);

        System.out.printf("WRN %15.3f : %s\n", nowutime / 1000000.0 , spk.message);
    }

    public static void main(String args[])
    {
        new RobotDriver();
    }
}
