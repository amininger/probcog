package probcog.robot.radio;

import java.util.regex.*;
import java.util.*;
import java.io.*;

import lcm.lcm.*;
import april.util.*;
import april.config.*;

import probcog.util.*;

/** Manages the transmission and reception of LCM message/channel
 * tuples over a radio with a policy for dropping messages when
 * packets are queued faster than they can be transmitted.
 *
 * All local LCM traffic ending in _TX will appear on all other nodes
 * in range as _ID. Note that the command control computer has a fixed
 * ID of 255.
**/
public class LCMRadio
{
    static final int MAGIC = 0x2756ee81;

    static LCM lcm = LCM.getSingleton();
    static Config config = Util.getConfig();

    static final boolean verbose = false;

    Radio radio;
    TransmitThread xmit = new TransmitThread();
    int nodeID;
    String suffix;
    final String RELAY_PREFIX = config.getString("lcm.relay_prefix", "RRR");

    public LCMRadio(Radio _radio, int nodeID, String _suffix)
    {
        this.nodeID = nodeID;
        this.radio = new FragmentingRadio(_radio, nodeID);
        this.suffix = _suffix;

        xmit.start();

        MyListener listener = new MyListener();
        String channel = ".*"+suffix;
        lcm.subscribe(channel, listener);
        radio.addListener(listener);

        System.out.println("NFO: Subscribing on channel "+channel);
    }

    class MyListener implements LCMSubscriber, RadioListener
    {
        // FROM LCM TO XTEND
        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            Matcher matcher = Pattern.compile(suffix).matcher(channel);
            if (!matcher.find())
                return;

            channel = channel.substring(0, channel.length() - matcher.group().length());

            try {
                LCMDataOutputStream outs = new LCMDataOutputStream();
                outs.writeInt(MAGIC);
                outs.writeStringZ(channel);

                byte msg[] = new byte[ins.available()];
                ins.readFully(msg);
                outs.write(msg);

                byte d[] = outs.toByteArray();

                if (d.length > radio.getMaxMessageSize()) {
                    System.out.println("WRN: Dropping large packet on '"+channel+"'");
                    return;
                }

                xmit.enqueue(channel, d);
            } catch (IOException ex) {
                System.out.println("WRN: "+ex);
            }
        }

        // FROM XTEND TO LCM
        // srcNodeID is correct, when coming from Fragmenting Radio.
        public void packetReceived(Radio radio, int srcNodeID, byte data[])
        {
            LCMDataInputStream ins = new LCMDataInputStream(data);
            try {
                int m = ins.readInt();
                if (m != MAGIC)
                    throw new IOException(String.format("bad magic %08x", m));

                // read channel name
                String channel = ins.readStringZ();

                // if a relay packet, remove string and do not add sourceID
                if (channel.startsWith(RELAY_PREFIX)) {
                    channel = channel.substring(RELAY_PREFIX.length());
                    lcm.publish(channel, ins.getBuffer(),
                                ins.getBufferOffset(), ins.available());
                } else
                    lcm.publish(channel+"_"+srcNodeID, ins.getBuffer(),
                                ins.getBufferOffset(), ins.available());

            } catch (IOException ex) {
                System.out.println("WRN: "+ex);
            }
        }
    }

    static class PendingPacket
    {
        String channel;
        byte   data[];
        long   utime;
    }

    // XTend transmit thread.
    class TransmitThread extends Thread
    {
        ArrayList<PendingPacket> queue = new ArrayList<PendingPacket>();

        int successiveFails = 0;

        public void run()
        {
            while (true) {
                // Our algorithm:
                //
                // 1. Which channel has been waiting the longest to transmit?
                // 2. Transmit the newest message queued to transmit on that channel.
                // 3. Remove all messages on that channel from the queue.

                PendingPacket best = null;

                synchronized (queue) {

                    // Wait until the queue is non-empty.
                    if (queue.size() == 0) {
                        try {
                            queue.wait();
                        } catch (InterruptedException ex) {
                            continue;
                        }
                    }

                    // Find the oldest packet.
                    PendingPacket oldest = null;

                    for (int qidx = 0; qidx < queue.size(); qidx++) {
                        PendingPacket pp = queue.get(qidx);
                        if (oldest == null || pp.utime < oldest.utime) {
                            oldest = pp;
                        }
                    }

                    // Find the newest packet on the same channel (and
                    // remove all packets on that channel from the
                    // queue.)
                    for (int qidx = 0; qidx < queue.size(); qidx++) {
                        PendingPacket pp = queue.get(qidx);

                        if (pp.channel.equals(oldest.channel)) {
                            if (best == null || pp.utime > best.utime) {
                                best = pp;
                            }
                            queue.remove(qidx);
                            qidx--;
                            continue;
                        }
                    }
                }

                // Now, transmit our winning packet.
                boolean success = radio.sendPacket(best.data, 0, best.data.length);
                if (success)
                    successiveFails = 0;
                else {
                    successiveFails++;
                    if (successiveFails >= 10) {
                        System.out.println("ERR: Quitting due to "+successiveFails+" successive sendPacket failures");
                        System.exit(1);
                    }
                }
            }
        }

        void enqueue(String channel, byte d[])
        {
            PendingPacket pp = new PendingPacket();
            pp.channel = channel;
            pp.data = d;
            pp.utime = TimeUtil.utime();

            synchronized(queue) {
                queue.add(pp);
                queue.notifyAll();
            }
        }
    }
}
