package probcog.robot.robot;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.util.*;
import april.util.GetOpt;
import april.config.*;

import probcog.lcmtypes.*;
import probcog.util.Util;

public class EspeakDaemon extends Thread implements LCMSubscriber
{
    static LCM lcm = LCM.getSingleton();

    PriorityQueue<speak_t> queue;

    int priorityThresh;

    Config config = Util.getConfig();

    boolean initialized = false;

    public EspeakDaemon()
    {
        priorityThresh = config.requireInt("espeak.priority_thresh");
        setDaemon(true);
        queue = new PriorityQueue<speak_t>(10, new MyComparator());
        lcm.subscribe("SPEAK.*", this);
    }

    public void run()
    {
        System.out.println("NFO: Starting ESpeakDaemon with priority cut off = "+priorityThresh);
        speak_t s = null;

        while(true) {
            System.out.println("DBG: INIT = " + initialized);
            synchronized(queue) {
                while (!initialized || (s = queue.poll()) == null) {
                    System.out.println("DBG: init = " + initialized);
                    try {
                        queue.wait();
                    }
                    catch(InterruptedException  e){}
                }

                // 1) Skip message of priority above thresh
                if (s.priority > priorityThresh) {
                    System.out.printf("NFO: Discarding espeak message \"%s\" because"+
                                      " priority is too big %d \n", s.message, s.priority);
                    continue;
                }

                // 2) Skip all status messages except the last one
                if (s.priority == priorityThresh && queue.size() != 0) {
                    System.out.printf("NFO: Discarding espeak STATUS message \"%s\" because"+
                                      " priority it is irrelevant \n", s.message);
                    continue;
                }
            }

            speakCommand(s);
            System.out.println("NFO: " + s.message + " [priority: " + s.priority+ "]");
        }
    }

    public void speakCommand(speak_t s)
    {
        if (s.voice == null || s.voice.length() == 0)
            s.voice = "en-us";

        String voice = s.voice;
        String args = "-s 150 ";

        String subs[] = config.getStrings("espeak.pronunciation", null);
        if (subs != null) {
            for (int i = 0; i+1 < subs.length; i+=2) {
                s.message = s.message.replaceAll(subs[i], subs[i+1]);
            }
        }

        if (s.voice.contains(":")) {
            String toks[] = s.voice.split(":");
            voice = toks[0];
            for (int i = 1; i < toks.length; i++) {
                args += String.format("-%c %s ", toks[i].charAt(0), toks[i].substring(1));
            }
        }

        String cmd[] = {"sh", "-c", String.format("espeak %s -v %s '%s' --stdout | aplay -q",
                                                  args, voice, s.message)};

        System.out.println(cmd[2]);
        try {
            Process p = Runtime.getRuntime().exec(cmd);
            try {
                p.waitFor();
                p.destroy(); // Cleaning up file descriptors
            } catch(InterruptedException e) {
            }
        } catch (IOException e) {
            System.out.println("WRN: Unable to speak");
        }
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(channel, ins);
        } catch (IOException ex) {
            System.out.println("Exception: "+ex);
        }
    }

    void messageReceivedEx(String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.startsWith("SPEAK")) {
            speak_t s = new speak_t(ins);
            synchronized (queue) {
                if (queue.offer(s) )
                    queue.notify();
            }
        }
    }

    /**
     * This class allows sorting queue of speak_t commands
     **/
    public class MyComparator implements Comparator
    {
        public int compare(Object o1, Object o2)
        {
            speak_t spk_1 = (speak_t) o1;
            speak_t spk_2 = (speak_t) o2;
            int priority_diff  = spk_1.priority - spk_2.priority;
            long time_diff  = spk_1.utime - spk_2.utime;

            // Sort on priority first, if priority is the same, then sort on time
            if (priority_diff > 0) {
                return 1;
            } else if (priority_diff < 0) {
                return -1;
            } else { // Compare on times instead of priority since priority is the same
                return (time_diff == 0 ? 0 : (time_diff > 0? 1:-1));
            }
        }
    }

    /**
     * Static functions for applications to call to send a speak message
     */
    public static void speak(String msg)
    {
        speak(LCM.getSingleton(), msg, "", 2);
    }

    public static void speak(String msg, String voice, int priority)
    {
        speak(LCM.getSingleton(), msg, voice, priority);
    }

    public static void speak(LCM _lcm, String msg, String voice, int priority)
    {
        if (msg == null)
            return;
        speak_t s = new speak_t();
        s.utime = TimeUtil.utime();
        s.message = msg;
        s.voice = voice;
        s.priority = priority;
        _lcm.publish("SPEAK", s);
    }

    public static void main(String args[])
    {
        GetOpt opts  = new GetOpt();

        opts.addBoolean('h',"help",false,"See this help screen");
        opts.addBoolean('g',"gc",false,"Ground Control");

        if (!opts.parse(args))
            System.out.println("option error: "+opts.getReason());
        if (opts.getBoolean("help")){
            System.out.println("Usage:");
            opts.doHelp();
            System.exit(1);
        }
        boolean gc = opts.getBoolean("gc");
        EspeakDaemon ed = new EspeakDaemon();
        String str;
        new Thread(ed).start();

        //if (gc)
        //    str = "Ground Control Started";
        //else
        //    str = "Robot " + RobotUtil.getID() + " Started";
        str = "Prob Cog Robot Started";

        EspeakDaemon.speak(str, "", 0);

    }
}
