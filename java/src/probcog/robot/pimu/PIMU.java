package magic.pimu;

import java.awt.*;
import java.util.*;
import java.io.*;

import april.jserial.*;
import april.jmat.*;
import april.util.*;

import magic.lcmtypes.*;

import lcm.lcm.*;

public class PIMU
{
    JSerial js;
    ReaderThread readerThread;
    ArrayList<PIMUListener> listeners = new ArrayList<PIMUListener>();

    pimu_t lastData;

    String params;
    GetOpt gopt;

    static int TIMEOUT_MS = 500;

    public static class Params
    {
        public int start_signature;
        public int version;
        public int length;

        public double gyro_scale[];
        public double gyro_offset[];

        public int end_signature;

        int swap4(int v)
        {
            return ((v&0xff)<<24) |
                (((v>>8)&0xff)<<16) |
                (((v>>16)&0xff)<<8) |
                (((v>>24)&0xff)<<0);
        }

        long swap8(long v)
        {
            return
                ((v&0xff)<<56) |
                (((v>>8)&0xff)<<48) |
                (((v>>16)&0xff)<<40) |
                (((v>>24)&0xff)<<32) |
                (((v>>32)&0xff)<<24) |
                (((v>>40)&0xff)<<16) |
                (((v>>48)&0xff)<<8) |
                (((v>>56)&0xff)<<0);
        }

        public Params(DataInputStream ins) throws IOException
        {
            start_signature = swap4(ins.readInt());
            version = swap4(ins.readInt());
            length = swap4(ins.readInt());
            gyro_scale = new double[8];
            gyro_offset = new double[8];
            for (int i = 0; i < 8; i++)
                gyro_scale[i] = Double.longBitsToDouble(swap8(ins.readLong()));
            for (int i = 0; i < 8; i++)
                gyro_offset[i] = Double.longBitsToDouble(swap8(ins.readLong()));
            end_signature = swap4(ins.readInt());
        }

        public void print()
        {
            System.out.printf("signature0: %08x\n", start_signature);
            System.out.printf("version   : %08x\n", version);
            System.out.printf("length    : %08x\n", length);

            for (int i = 0; i < 8; i++) {
                System.out.printf("axis %d, gain %.3f, offset %.3f\n",
                                  i,
                                  gyro_scale[i],
                                  gyro_offset[i]);
            }

            System.out.printf("signature1: %08x\n", end_signature);
        }

        public byte[] toByteArray()
        {
            try {
                ByteArrayOutputStream bouts = new ByteArrayOutputStream();
                DataOutputStream outs = new DataOutputStream(bouts);
                outs.writeInt(swap4(start_signature));
                outs.writeInt(swap4(version));
                outs.writeInt(swap4(length));
                for (int i = 0; i < 8; i++)
                    outs.writeLong(swap8(Double.doubleToLongBits(gyro_scale[i])));
                for (int i = 0; i < 8; i++)
                    outs.writeLong(swap8(Double.doubleToLongBits(gyro_offset[i])));
                outs.writeInt(swap4(end_signature));
                outs.flush();

                return bouts.toByteArray();
            } catch (IOException ex) {
                System.out.println("ex: "+ex);
                return null;
            }
        }
    }

    public interface PIMUListener
    {
        public void pimuData(pimu_t d);
    }

    // for handling synchronous commands
    Object responseNotify = new Object();
    byte request[];
    byte response[];

    TimeSync timeSync = new TimeSync(1E6, 0, 0.001, 0.5);

    void writePacket(byte buf[])
    {
        // magic header
        js.write(0xed);
        js.write(0x87);
        js.write(0x17);
        js.write(0x9a);

        int chk = checksum_init();

        // output payload length
        int len = buf.length;

        js.write(len>>8);
        chk = checksum_update(chk, len >> 8);
        js.write(len & 0xff);
        chk = checksum_update(chk, len & 0xff);

        // output payload
        js.write(buf, 0, buf.length);
        for (int i = 0; i < len; i++) {
            chk = checksum_update(chk, buf[i]);
        }

        // output checksum
        chk = checksum_finish(chk);
        js.write(chk);
    }

    byte[] readMessage(JSerial js) throws IOException
    {
        while (true) {

            // synchronize to magic string.
            while (true) {
                int b = js.read();
                if (b < 0) {
                    throw new EOFException();
                }

                if (b != 0xed)
                    continue;
                b = js.read();
                if (b != 0x87)
                    continue;
                b = js.read();
                if (b != 0x71)
                    continue;
                b = js.read();
                if (b != 0xe2)
                    continue;
                break;
            }

            int chk = checksum_init();

            int len = ((js.read()&0xff)<<8) + (js.read()&0xff);
            chk = checksum_update(chk, len>>8);
            chk = checksum_update(chk, len & 0xff);

            if (len > 1024)
                continue;

            // build a buffer that contains the payload
            byte buf[] = new byte[len];

            js.readFully(buf, 0, len); // don't read header, len, or chk
            for (int i = 0; i < len; i++)
                chk = checksum_update(chk, buf[i]);

            chk = checksum_finish(chk);

            int readChk = js.read()&0xff;

            if (readChk != chk) {
                long mtime = System.currentTimeMillis();
                System.out.printf("bad: %d\n", mtime - last_mtime);
                last_mtime = mtime;
/*
                System.out.printf("WRN: Bad pimu packet (len=%d, %02x != %02x) ", len, readChk, chk);
                for (int i = 0; i < buf.length; i++) {
                    System.out.printf("%02x ",  buf[i] & 0xff);
                }
                System.out.println("");
*/              continue;
            }

            return buf;
        }
    }

    long last_mtime;

    static int checksum_init()
    {
        return 0x12345678;
    }

    static int checksum_update(int chk, int c)
    {
        chk = chk + (c & 0xff);
        chk = (chk << 1) ^ (chk >> 23);
        return chk;
    }

    static int checksum_finish(int chk)
    {
        return (chk & 0xff) ^ ((chk >> 8) & 0xff) ^ ((chk >> 16) & 0xff);
    }

    public PIMU(GetOpt gopt)
    {
        this.gopt = gopt;

        readerThread = new ReaderThread();
        readerThread.start();
    }

    public pimu_t getLast()
    {
        return lastData;
    }

    public synchronized void addListener(PIMUListener listener)
    {
        listeners.add(listener);
    }

    public synchronized void removeListener(PIMUListener listener)
    {
        listeners.remove(listener);
    }

    public byte[] doCommand(byte b[], int timeoutms)
    {
        synchronized (responseNotify) {

            response = null;
            request = b;

            try {
                writePacket(b);
            } catch (Exception ex) {
            }

            try {
                responseNotify.wait(timeoutms);
            } catch (InterruptedException ex) {
            }

            return response;
        }
    }

    public Params getParams()
    {
        while (true) {
            try {
                byte r[] = doCommand(new byte[] { 10, 0 }, TIMEOUT_MS);
                if (r == null)
                    continue;
                return new Params(new DataInputStream(new ByteArrayInputStream(r, 2, r.length - 2)));
            } catch (IOException ex) {
                System.out.println("ex: "+ex);
            }
        }
    }

    public void setParams(Params p)
    {
        try {
            ByteArrayOutputStream bouts = new ByteArrayOutputStream();
            DataOutputStream outs = new DataOutputStream(bouts);

            outs.write(20);
            outs.write(0);
            outs.write(p.toByteArray());
            outs.flush();

            byte r[] = doCommand(bouts.toByteArray(), TIMEOUT_MS);
            if (r == null || r[1] != 0)
                System.out.println("setParams failed");
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }

    class ReaderThread extends Thread
    {
        public void run()
        {
            while (true) {

                try {
                    js = new JSerial(gopt.getString("device"), 230400, "8N1", true);
                } catch (IOException ex) {
                    System.out.println("ex: "+ex);
                    try {
                        Thread.sleep(1000);
                    } catch (InterruptedException ex2) {
                    }
                    continue;
                }

                while (true) {

                    try {
                        byte buf[] = readMessage(js);

                        // data packet!
                        if (buf[0] == 1) {

                            DataInputStream ins = new DataInputStream(new ByteArrayInputStream(buf, 2, buf.length - 2));

                            pimu_t pimu = new pimu_t();

                            pimu.utime_pimu = ins.readLong();
                            long utime = TimeUtil.utime();
                            timeSync.update(utime, pimu.utime_pimu);
                            pimu.utime = timeSync.getHostUtime(pimu.utime_pimu);

                            for (int i = 0; i < 8; i++)
                                pimu.integrator[i] = ins.readInt();
                            for (int i = 0; i < 3; i++)
                                pimu.accel[i] = ins.readShort();
                            for (int i = 0; i < 3; i++)
                                pimu.mag[i] = ins.readShort();
                            for (int i = 0; i < 2; i++)
                                pimu.alttemp[i] = ins.readInt();

                            assert(ins.available() == 0);
                            synchronized(listeners) {
                                lastData = pimu;

                                for (PIMUListener listener : listeners) {
                                    listener.pimuData(pimu);
                                }
                            }

                        } else if ((buf[0]&0xff) == 255) {
                            // debug string
                            System.out.print("PIMU DEBUG: ");
                            for (int i = 2; i < buf.length; i++) {
                                System.out.printf("%c", buf[i]);
                            }

                        } else {
                            // some sort of command
                            synchronized (responseNotify) {
                                if (request != null && buf[0] == request[0]) {
                                    response = buf;
                                    responseNotify.notifyAll();
                                }
                            }
                        }
                    } catch (IOException ex) {
                        System.out.println("Ex: "+ex);
                        js.close();
                        break;
                    }
                }
            }
        }
    }

    static class LCMPublisher extends Thread implements PIMUListener
    {
        LCM lcm = LCM.getSingleton();
        long lastutime;
        GetOpt gopt;
        int rxCount, txCount;

        LCMPublisher(GetOpt gopt)
        {
            this.gopt = gopt;
            this.start();
        }

        public void pimuData(pimu_t pimu)
        {
            rxCount++;

            long utime = TimeUtil.utime();
            double dt = (utime - lastutime) / 1000000.0;
            if (gopt.getInt("hz")!=0 && dt < 1.0 / gopt.getInt("hz"))
                return;
            lastutime = utime;

            lcm.publish("PIMU", pimu);
            txCount++;
        }

        public void run()
        {
            while (true) {
                long utime0 = TimeUtil.utime();
                TimeUtil.sleep(1000);
                long utime1 = TimeUtil.utime();
                double dt = (utime1 - utime0) / 1000000.0;
                System.out.printf("%15.3f : PIMU rx=%.1f Hz, tx=%.1f Hz\n", utime1 / 1000000.0,
                                  rxCount / dt, txCount / dt);
                rxCount = 0;
                txCount = 0;
            }
        }
    }

    public static void main(String args[])
    {
        GetOpt gopt = new GetOpt();
        gopt.addInt('\0', "hz", 0, "Max publish rate (Hz). Use 0 for all.");
        gopt.addString('d', "device", "/dev/pimu", "Device path");
        gopt.addBoolean('h', "help", false, "Show this help");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            gopt.doHelp();
            System.exit(-1);
        }

        PIMU imu = new PIMU(gopt);

        imu.addListener(new LCMPublisher(gopt));
    }
}
