package probcog.util;

import java.io.*;

import lcm.lcm.*;

import april.util.*;

import magic2.lcmtypes.*;

public class RemoteLCMTest
{
    static public void main(String[] args)
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show usage");
        gopt.addString('u', "url", "udpm://239.255.76.68:7668?ttl=1", "LCM URL");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            System.out.printf("Usage: %s [-u LCM-URL]\n", args[0]);
            gopt.doHelp();
            System.exit(1);
        }

        try {
            LCM lcm = new LCM(gopt.getString("url"));

            speak_t speak = new speak_t();
            speak.utime = TimeUtil.utime();
            speak.args = "";
            speak.priority = 3;
            speak.message = "LCM test message";

            lcm.publish("LCM_TEST", speak);
        } catch (IOException ex) {
            ex.printStackTrace();
        }
    }
}
