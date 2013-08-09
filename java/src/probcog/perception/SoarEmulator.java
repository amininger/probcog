package probcog.perception;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.util.*;

import probcog.lcmtypes.*;

public class SoarEmulator
{
    LCM lcm = LCM.getSingleton();
    observations_t obs;

    public SoarEmulator()
    {
        new ListenerThread().start();
    }

     /** Class that continually listens for messages from Soar about what objects
     *  it believes exists in the world. The received lcm message is stored so it
     *  can be used upon request.
     **/
    class ListenerThread extends Thread implements LCMSubscriber
    {

        public ListenerThread()
        {
            lcm.subscribe("OBSERVATIONS", this);
        }

        public void run()
        {
            while (true) {
                TimeUtil.sleep(100);

                if(obs != null) {
                    synchronized (obs) {
                        soar_objects_t soar = new soar_objects_t();
                        soar.utime = TimeUtil.utime();
                        soar.num_objects = obs.nobs;
                        soar.objects = new object_data_t[soar.num_objects];
                        for (int i = 0; i < soar.objects.length; i++) {
                            soar.objects[i] = obs.observations[i];
                        }
                        lcm.publish("SOAR_OBJECTS", soar);
                    }
                }
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
            if (channel.equals("OBSERVATIONS")) {
                obs = new observations_t(ins);
            }
        }
    }
}
