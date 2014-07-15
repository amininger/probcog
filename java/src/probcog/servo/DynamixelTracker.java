package probcog.servo;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jmat.*;

import probcog.lcmtypes.*;

public class DynamixelTracker implements LCMSubscriber
{
    String channel;

    // how long back in time should we remember poses?
    double time;
    double latency;

    LinkedList<dynamixel_status_t> queue = new LinkedList<dynamixel_status_t>();

    LCM lcm = LCM.getSingleton();

    static final int MAX_QUEUE_SIZE = 10000;
    boolean warned;

    public double maxTimeErr = 0.1;

    public DynamixelTracker(String channel)
    {
        this(channel, 5, 0.0235);
    }

    /** @param channel Which channel to track?
        @param time How far back in history should we remember?
        @param latency Assume utimes in Dynamixel messages are latency seconds older.
    **/
    public DynamixelTracker(String channel, double time, double latency)
    {
        this.channel = channel;
        this.time = time;
        this.latency = latency;

        lcm.subscribe(channel, this);
    }

    public synchronized dynamixel_status_t get()
    {
        return queue.getLast();
    }

    public dynamixel_status_t get(long utime)
    {
        dynamixel_status_t status0 = null, status1 = null; // two status messages bracketing utime.

        utime -= (long) (latency*1000000);

        synchronized(queue) {
            for (dynamixel_status_t st : queue) {
                if (st.utime < utime && (status0 == null || st.utime > status0.utime))
                    status0 = st;
                if (st.utime > utime && (status1 == null || st.utime < status1.utime))
                    status1 = st;
            }
        }
        if (status0 != null && Math.abs(utime - status0.utime) > maxTimeErr*1000000)
            status0 = null;

        if (status1 != null && Math.abs(utime - status1.utime) > maxTimeErr*1000000)
            status1 = null;

        if (status0 != null && status1 != null) {
            // interpolate based on time.
            double err0 = Math.abs(status0.utime - utime);
            double err1 = Math.abs(status1.utime - utime);

            double w0 = err1 / (err0 + err1);
            double w1 = err0 / (err0 + err1);

            dynamixel_status_t s = new dynamixel_status_t();
            s.utime = utime;
            s.error_flags = status0.error_flags | status1.error_flags;
            s.position_radians =  status0.position_radians*w0 +
                status1.position_radians*w1;
            s.speed = status0.speed*w0 + status1.speed*w1;
            s.load = status0.load*w0 + status1.load*w1;
            s.voltage = status0.voltage*w0 + status1.voltage*w1;
            s.temperature = status0.temperature*w0 + status1.temperature*w1;

            return s;
        }

        if (status0 != null)
            return status0;

        if (status1 != null)
            return status1;

        return null;
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            dynamixel_status_t s = new dynamixel_status_t(ins);

            synchronized(queue) {
                queue.add(s);

                // emergency shrinkage.
                while (queue.size() > MAX_QUEUE_SIZE) {
                    queue.removeFirst();
                    if (!warned) {
                        System.out.println("DynamixelTracker queue too large");
                        warned = true;
                    }
                }

                while (true) {
                    dynamixel_status_t first = queue.getFirst();
                    dynamixel_status_t last = queue.getLast();

                    if (Math.abs(last.utime - first.utime) > time*1000000)
                        queue.removeFirst();
                    else
                        break;
                }
            }
        } catch (IOException ex) {
            System.out.println("Exception: " + ex);
        }
    }
}
