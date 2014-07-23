package probcog.robot.radio;

import java.io.IOException;
import java.util.*;
import java.util.zip.CRC32;

// For convenience
import lcm.lcm.LCMDataInputStream;
import lcm.lcm.LCMDataOutputStream;

public class FragmentingRadio implements RadioListener, Radio
{
    static final int MAGIC = 0x0237EDED;

    Radio radio;
    ArrayList<RadioListener> listeners = new ArrayList<RadioListener>();

    // The partial plans are indexed by a binary concatenation of the
    // source address and the frame number
    HashMap<Long,PartialPacket> partials;

    // The message count (NOT the packet count).
    int messageCount;

    // The ID of this node, used to validate messages
    int nodeID;

    public static final int EXPECTED_OVERHEAD = 24; //In bytes.

    // fragmentationSize: For an input 'byte data[]' which exceeds
    // this threshold, it is split into multiple packets. This takes
    // into account the maximum possible overhead per packet
    // (introduced by this class), and the max packet size of the
    // underlying radio
    int fragmentationSize;

    public FragmentingRadio(Radio radio, int _nodeID)
    {
        this.radio = radio;
        this.radio.addListener(this);

        this.partials = new HashMap<Long, PartialPacket>();

        nodeID = _nodeID;

        fragmentationSize = radio.getMaxMessageSize() - EXPECTED_OVERHEAD;
        new ReaperThread().start();
    }

    public int getMaxMessageSize()
    {
        return 10*fragmentationSize;
    }

    public static void hexDump(byte data[])
    {
        System.out.printf("\n%04d\t",0);
        for(int i=0; i < data.length; i++)
        {
            System.out.printf("%02x ", data[i]&0xFF);
            if((i+1)%8 == 0)
                System.out.print("    ");
            if((i+1)%16 == 0)
                System.out.printf("\n%04d:\t",i+1);
        }
        System.out.println();
    }

    public boolean sendPacket(byte input[], int offset, int datalen)
    {
        byte data [] = new byte[datalen];
        System.arraycopy(input,offset,data,0,datalen);
        return sendPacket(data);
    }

    public boolean sendPacket(byte data[])
    {
        boolean success = true;
        int offset = 0;

        int newMC = messageCount++;

        while(offset < data.length){
            // Marshall the LCM message into a radio packet.
            LCMDataOutputStream outs = new LCMDataOutputStream();

            outs.writeInt(MAGIC);
            outs.writeInt(nodeID);
            outs.writeInt(newMC);
            outs.writeInt(data.length); // total packet length

            if (data.length > fragmentationSize)
                outs.writeInt(offset);

            if (offset == 0) {
                //NO channel name in message
                CRC32 crc = new CRC32();
                crc.update(data);
                outs.writeInt((int) crc.getValue());
            }

            int writesz = Math.min(fragmentationSize, data.length - offset);
            outs.write(data, offset, writesz);
            offset += writesz;

            byte [] buffer = outs.toByteArray();

            success = success && radio.sendPacket(buffer, 0, buffer.length);
        }

        return success;
    }

    public void addListener(RadioListener listener)
    {
        listeners.add(listener);
    }

    //This Method is called when the XTend has a packet for us.
    // Note: sourceAddr is erroneous for XTendRadios
    public void packetReceived(Radio radio, int sourceAddr, byte data[])
    {
        LCMDataInputStream ins = new LCMDataInputStream(data);
        try {
            int magic = ins.readInt();
            if (magic != MAGIC)
                return;

            int srcNodeID = ins.readInt();
            int frame = ins.readInt();
            int datalen = ins.readInt();
            int offset = 0;
            if (datalen > fragmentationSize)
                offset = ins.readInt();
            int crcchk = 0;
            if (offset == 0) {
                crcchk = ins.readInt();
            }

            PartialPacket part = null;
            synchronized(partials) {
                if (datalen <= fragmentationSize) {
                    part = new PartialPacket(srcNodeID, frame, datalen);
                } else if (part == null) {
                    part = partials.remove(getCombinedKey(srcNodeID, frame));
                    if (part == null) {
                        part = new PartialPacket(srcNodeID, frame, datalen);
                    }
                }

                if (offset == 0) {
                    part.setCRC32(crcchk);
                }
                part.addData(ins,offset,datalen);

                if (!part.isDone()) {
                    //more work to do
                    partials.put(part.getKey(), part);
                }
            }

            // After synchronized, do callbacks
            if (part.isDone()) {
                //Packet is done, lets notify radio listeners
                byte [] part_data = part.getData();

                // if CRC check fails, getData should return null
                if (part_data == null)
                    return;

                for (RadioListener listener : listeners)
                    listener.packetReceived(radio, srcNodeID, part_data);
            }

        }catch (IOException ex) {
            System.out.println("Ex: "+ex);
            return;
        }
    }

    class ReaperThread extends Thread
    {
        int counter;
        public void run()
        {
            while (true) {

                synchronized(partials) {
                    ArrayList<Long> remove_keys = new ArrayList<Long>();
                    // find old packets
                    for (Long key: partials.keySet()) {
                        if (partials.get(key).isStale())
                            remove_keys.add(key);
                    }

                    // Remove old packets
                    for (Long key : remove_keys) {
                        partials.remove(key);
                        counter++;
                        System.out.println("NFO: FragmentingRadio.ReaperThread: Removing stale packet # "+counter);
                    }
                }

                // sleep
                try {
                    Thread.sleep(1000);
                } catch(InterruptedException e) {
                    System.out.println("WRN: Reaper Ex: "+e);
                }
            }
        }
    }

    static long getCombinedKey(int src, int frm)
    {
        long source = src;
        long frame = frm;

        return (source << 32) | (frame & 0xffffffff);
    }

    class PartialPacket
    {
        // maximum staleness of package before reaping
        public int STALE_TIME_THRESH_MS = 4000;

        // milliseconds since last access
        long lastAccessMS;

        // (approximately) globally-unique message ID
        long key;

        // CRC32 of whole message (received in first fragment)
        int crc32;

        // the packet itself
        byte data[];

        // flags indicating reception of each of the packet's fragments.
        boolean recieved[];
        // num non zero packets got so far
        int recievedNZ;

        boolean failedCRC;

        public PartialPacket(int _source, int _frameCount, int _datalen)
        {
            key = FragmentingRadio.getCombinedKey(_source, _frameCount);
            touch();

            data = new byte[_datalen];

            int numDiv = (int) Math.ceil(((double) _datalen) / fragmentationSize);
            numDiv = Math.max(1, numDiv);

            recieved = new boolean[numDiv];
            recievedNZ = 0;
        }

        long getKey()
        {
            return key;
        }

        private void touch()
        {
            lastAccessMS = System.currentTimeMillis();
        }

        void setCRC32(int _crc32)
        {
            crc32 = _crc32;
        }

        void addData(LCMDataInputStream ins, int datapos, int _datalen) throws IOException
        {
            // Check for invalid association
            if (data.length != _datalen) {
                System.out.println("WRN: Partial packetization association on source and frame failed on datalen check.");
                System.out.printf("WRN: Expected datalen %d found %d\n", data.length, _datalen);
                return;
            }

            // how much data should this packet contain?
            int dlen = Math.min(fragmentationSize, data.length - datapos);
            if (dlen != ins.available()) {
                System.out.println("WRN: Available bytes from input stream does not match expected.");
                System.out.printf("          Available = %d, expected min(%d,%d)\n",
                                  ins.available(), fragmentationSize, data.length - datapos);
                return;
            }

            ins.readFully(data, datapos, dlen);

            int slot = datapos / fragmentationSize;

            if (!recieved[slot]) {
                recieved[slot] = true;
                recievedNZ++;
            }

            if (recievedNZ == recieved.length) {
                // We've received everything.
                CRC32 crc = new CRC32();
                crc.update(data);
                failedCRC = crc32 != ((int) crc.getValue());
                if (failedCRC) {
                    System.out.println("WRN: Partial Packet reconstruction failedCRCed due to CRC check: expected "+crc32+" but got "+((int) crc.getValue()));
                    // we could return here, but harmless not to.
                }
            }
            touch();
        }

        boolean isDone()
        {
            return recievedNZ == recieved.length;
        }

        byte[] getData()
        {
            if (isDone() && !failedCRC)
                return data;

            return null;
        }

        boolean isStale()
        {
            return (System.currentTimeMillis() - lastAccessMS) > STALE_TIME_THRESH_MS;
        }
    }
}
