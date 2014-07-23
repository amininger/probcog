package probcog.robot.radio;

import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.jserial.*;
import april.lcmtypes.*;
import april.util.TimeUtil;

import probcog.lcmtypes.*;
import probcog.util.*;
//import magic.robot.EspeakDaemon;
//import magic.util.Util;
//import magic.lcmtypes.*;

public class XTendRadio implements Radio
{
    static LCM lcm = LCM.getSingleton();

    public static final int FIRMWARE_US = 2067;
    public static final int FIRMWARE_AU = 2268;

    private static final int START_IDX = 7; //For indexing payload

    public static final int BROADCAST = 0xffff;

    ArrayList<RadioListener> listeners = new ArrayList<RadioListener>();

    JSerial js;

    // don't start with zero: causes no ACK to be sent.
    int txFrameNumber = 1;

    // mutex that guards txAckFrameNumber
    Object txFrameNumberAckObject = new Object();

    // last acknowledge frame number.
    int txFrameNumberAck = -1;

    // timeout between characters within a single response.
    static final int INTRAPACKET_TIMEOUT_MS = 100;

    // maximum delay when waiting for tx acknowledgement from
    // xtend. Needs to be long enough for XTend to successfully
    // negotiate a time slot? Could be long if network is busy?
    static final int TX_ACK_TIMEOUT_MS = 2000;

    static final int MAX_PACKET_SIZE = 2000; //In bytes

    final int destAddr;
    final byte nodeID;
    private int hopPlan;  // only HopSequenceThread should modify

    int msgNum; //valid values: [0, ... , 127]

    boolean inCommandMode;

    long cmdsUtime; // needed to properly handle CMDS message
    boolean ensureMonotonicMsgs;  // only accept newer msge (no loops in log playback)

    //BandwidthReporter reporter;

    /**
     * Constructor
     **/
    public XTendRadio(String path, byte _nodeID, int destAddr, int _hopPlan,
                      int firmware) throws IOException
    {
        this(path, _nodeID, destAddr, _hopPlan, firmware, false);
    }

    /**
     * Constructor
     * @path e.g. "/dev/ttyUSB0"
     * @destAddr 0xffff = bcast
     * @_hopPlan  0-9
     * @firmware  pick to enforce one of FIRMWARE_AU or FIRMWARE_US
     * @dynamicHopSeq radio can switch hopping sequences
     **/
    public XTendRadio(String path, byte _nodeID, int destAddr, int _hopPlan,
                      int firmware, boolean dynamicHopSeq) throws IOException
    {
        this.ensureMonotonicMsgs = Util.getConfig().getBoolean("lcm.accept_only_monotonic_utimes", true);

        nodeID = _nodeID;
        this.destAddr = destAddr;

        System.out.printf("NFO: Initializing radio with id %d at %s\n", _nodeID, path);
        // Autobaud.
        int bauds[] = new int[] {230400, 9600 };
        // We *know* that the radio should never be in the following baud rates:
        //                                  115200, 57600, 38400, 19200,
        //                                  4800, 2400, 1200 };
        // (so we don't try them)

        js = new JSerial(path, bauds[0], "8N1");

        for (int bidx = 0; true; bidx++) {
            int baud = bauds[bidx % bauds.length];

            System.out.printf("NFO: [%4d] Trying baud %d...", bidx, baud);
            System.out.flush();
            js.setBaud(baud);

            if (enterCommandMode()) {
                break;
            }
        }

        System.out.printf(" found!\n");

        /////////////////////////////////////////////////
        // Read firmware version. If it doesnt match expected, exit
        String vr = doATRead("ATVR");
        if (!vr.equals(""+firmware)) {
            System.err.printf("\nERR: Found firmware version %s, expected %d\n", vr, firmware);
            System.exit(1);
        }

        // Set API mode
        doATCommand("ATAP=1", true);

        // Set transmit power to (3=500mW, 4 = 1W)
        doATCommand("ATPL=4", true);

        // Set RF baud rate to (0=9600, 1 = 115200)
        doATCommand("ATBR=1", true);

        // Set interface baud rate to (7=115200, 8=230400)
        // Will take effect when we finish.
        doATCommand("ATBD=8", true);

        // Disable packet send retries
        doATCommand("ATRR=0", true);

        // Set channel hopping schedule.
        doATCommand("ATHP="+_hopPlan, true);
        this.hopPlan = _hopPlan;

        // Exit command mode. Baud rate will take effect upon completion.
        // Beware of extra data in buffer recieved from other radios
        // when we leave command mode
        doATCommand("ATCN", false);

        js.setBaud(230400);

        System.out.println("NFO: Initialization complete");

        //reporter = new BandwidthReporter();
        //reporter.start();
        new ReaderThread().start();

        //if (dynamicHopSeq)
        //    new HopSequenceThread(_hopPlan).start();
    }

    private boolean enterCommandMode()
    {
        byte buf[] = new byte[65536];

        // must not send anything else for > 1s before sending +++.
        TimeUtil.sleep(1250);

        // enter command mode
        js.write("+++");

        // must not send anything else for > 1s. The modem will respond "OK"
        int len = readFor(buf, 1250);
        if (len == 0) {
            System.out.printf("Read Nothing\n");
            return false;
        }

        if (len == buf.length) {
            System.out.printf("Filled discard buffer!\n.");
            return false;
        }

        // is there an OK somewhere in this buffer?
        String s = new String(buf, 0, len);

        System.out.printf("len : "+len);

        if (s.endsWith("OK\r")) {

            System.out.printf("(in data mode?)");
            System.out.flush();

        } else {
            // maybe we were already in command mode. Try sending
            // an AT (we should get an OK back)

            // double check by sending a new AT
            js.write("AT\r");
            len = readFor(buf, 250);
            s = new String(buf, 0, len);

            // if not OK, then unsuccessful.
            if (!s.equals("OK\r")) {
                return false;
            }

            System.out.printf("(in command mode?)");
            System.out.flush();
        }

        // either way, double check that we're really in command
        // mode by sending another AT.
        js.write("AT\r");
        len = readFor(buf, 250);
        s = new String(buf, 0, len);

        // if OK, command mode entered
        if (s.equals("OK\r"))
            return true;

        return false;
    }

    /**
     * Thread added to handle requests to change the hopping sequence
     * of radio.  Needed to prevent infinite lockup of messageReceived
     * function.
     **/
    /*class HopSequenceThread extends Thread implements LCMSubscriber
    {
        private int desiredHopPlan;

        public HopSequenceThread(int _hopPlan)
        {
            this.desiredHopPlan = _hopPlan;
            int gcsID = Util.getGNDID();
            System.out.println("NFO: Subscribing to CMDS_"+gcsID);
            lcm.subscribe("CMDS_"+gcsID, this);
        }

        public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
        {
            try {
                messageReceivedEx(lcm, channel, ins);
            } catch (IOException ex) {
                System.out.println("WNG: IOException: "+ex);
            }
        }

        public void messageReceivedEx(LCM lcm, String channel, LCMDataInputStream ins)
            throws IOException
        {
            if (channel.equals("CMDS_"+Util.getGNDID())){
                robot_command_list_t commandList = new robot_command_list_t(ins);
                if (!ensureMonotonicMsgs || cmdsUtime < commandList.utime) {
                    cmdsUtime = commandList.utime;
                    if (commandList.aux != null) {
                        int newHopPlan = -1;
                        for (int i = 0; i < commandList.aux.nrobots; i++) {
                            if (commandList.aux.xtend_hop_plan[i][0] == nodeID) {
                                newHopPlan = commandList.aux.xtend_hop_plan[i][1];
                                break;
                            }
                        }
                        if (newHopPlan != -1){
                            synchronized (this) {
                                this.desiredHopPlan = newHopPlan;
                                this.notify();
                            }
                        }
                    }
                }
            }
        }
        public void run()
        {
            int newHopPlan;
            while (true) {
                synchronized (this) {
                    do {
                        newHopPlan = desiredHopPlan;
                        try {
                            this.wait(250);
                        } catch (InterruptedException ex) {
                        }
                    }
                    while(newHopPlan == hopPlan);
                }
                try {
                    setHopSequence(newHopPlan);
                } catch (IOException ex) {
                    System.out.println("WRN: "+ex);
                    System.exit(-1);
                }
            }
        }

        private void setHopSequence(int _hopPlan)  throws IOException {
            // valid number and change
            if (hopPlan == _hopPlan || inCommandMode || _hopPlan < 0 || _hopPlan > 9)
                return;

            // Disable sending packets
            inCommandMode = true;
            System.out.println("NFO: Changing hop sequence, send/read DISABLED");

            int tries = 0;
            while (true) {
                System.out.printf("NFO: [%4d] Trying to enter command mode...", tries++);
                if (enterCommandMode())
                    break;
            }
            System.out.println("\tsuccess");

            // Set channel hopping schedule.
            doATCommand("ATHP="+_hopPlan, true);
            hopPlan = _hopPlan;

            // Exit command mode. Baud rate will take effect upon completion.
            // Beware of extra data in buffer recieved from other radios
            // when we leave command mode
            doATCommand("ATCN", false);

            // Re-enable sending packets
            inCommandMode = false;
            //String str = "Robot " + nodeID + " hop plan " + _hopPlan;
            //System.out.println("NFO: "+str + ": send/read ENABLED");
            //EspeakDaemon.speak(str);
        }
    }*/

    /**
     * In API mode, we can only send packets of 2000 bytes
     */
    public int getMaxMessageSize()
    {
        return MAX_PACKET_SIZE;
    }
    /** Perform an AT command (which lacks the terminating \r). Must
     * already be in command mode. Waits for an OK and returns. **/
    void doATCommand(String atcmd, boolean paranoid)
    {
        int trials = 0;

        while(true) {
            System.out.printf("NFO: [%4d] Command: %s\n", trials, atcmd);

            js.write(atcmd+"\r");
            byte buf[] = new byte[1024];
            int len = readFor(buf, 250);
            String s = new String(buf, 0, len);
            if (paranoid) {
                if (s.equals("OK\r"))
                    return;
            } else {
                if (s.startsWith("OK\r"))
                    return;
            }

            trials++;

            if (trials > 10)
                fatal();
        }
    }

    /** Perform an AT read command (lacking the terminating \r). Must
     * already be in command mode. Waits for value and returns it. **/
    String doATRead(String atcmd)
    {
        int trials = 0;

        while(true) {
            System.out.printf("NFO: [%4d] Read: %s=", trials, atcmd);

            js.write(atcmd+"\r");
            byte buf[] = new byte[1024];
            int len = readFor(buf, 250);
            if (len != 0 && buf[len-1] == '\r') {
                String s = new String(buf, 0, len -1);
                System.out.printf("%s\n",s);
                return s;
            }
            if (len == 0)
                System.out.println("[nothing]");
            else
                System.out.printf("%s\n", new String(buf, 0, len));

            trials++;

            if (trials > 10)
                fatal();
        }
    }

    // Read for a fixed amount of time, returning all of the contents
    // seen during that time. This is useful during auto-negotiation,
    // since if there is queued up junk in the RX buffer, we'll get
    // it, thus helping us notice that we're out of sync.
    int readFor(byte buf[], int ms)
    {
        long endtime = System.currentTimeMillis()+ms;
        int offset = 0;
        int len = 0;

        while (true) {
            int msleft = (int) (endtime - System.currentTimeMillis());
            if (msleft < 0)
                break;

            int thislen = js.readTimeout(buf, offset+len, buf.length - offset - len, msleft);
            len += thislen;
        }

        return len;
    }

    /** @param destAddr Note: broadcast address is 0xffff
     **/
    public boolean sendPacket(byte data[], int offset, int datalen)
    {
        if (datalen > MAX_PACKET_SIZE) {
            System.err.printf("ERR: Attempted to send packet of length %d, but limit is %d\n",
                              datalen, MAX_PACKET_SIZE);
            System.exit(1);
        }

        // Cannot send while in command mode
        boolean printOnce;
        while (inCommandMode) {
            if (false && !printOnce)
                System.out.println("NFO: Packet waiting to send"); // dah
            printOnce = true;
            TimeUtil.sleep(100);
        }

        // assert(datalen <= 2048);

        byte cmd[] = new byte[datalen + START_IDX];
        cmd[0] = 0x01; // TX
        cmd[1] = (byte) txFrameNumber;
        cmd[2] = (byte) ((destAddr>>8) & 0xff); // broadcast
        cmd[3] = (byte) (destAddr & 0xff); // broadcast
        cmd[4] = 0; // Standard

        // Our header
        cmd[5] = nodeID; // Standard
        cmd[6] = (byte)msgNum; // Standard

        msgNum++;
        if (msgNum > 127)
            msgNum = 0;

        // Our data
        for (int i = 0; i < datalen; i++)
            cmd[START_IDX + i] = data[offset + i];

        boolean success = false;

        synchronized (txFrameNumberAckObject) {
            try {
                rawCommandSend(cmd, 0, cmd.length);

                txFrameNumberAckObject.wait(TX_ACK_TIMEOUT_MS);

                if (txFrameNumberAck == txFrameNumber)
                    success = true;
            } catch (InterruptedException ex) {
            }
        }

        if (!success)
            System.out.println("WRN: XTendRadio send failed! expected frame: "+
                               txFrameNumber+", got frame: "+txFrameNumberAck);

        //reporter.reportTX(txFrameNumber, datalen);

        txFrameNumber++;
        if (txFrameNumber == 256) // don't wrap around to zero: radio won't send acks for frame#=0
            txFrameNumber = 1;


        return success;
    }

    /** Send a command to the XTend **/
    private synchronized void rawCommandSend(byte payload[], int offset, int payloadlen)
    {
        // calling function already checks if in command mode

        byte header[] = new byte[3];
        header[0] = 0x7e;
        header[1] = (byte) (payloadlen >> 8);
        header[2] = (byte) (payloadlen & 0xff);
        js.write(header, 0, 3);
        js.write(payload, offset, payloadlen);

        int chk = 0;
        for (int i = 0; i < payload.length; i++)
            chk += (payload[i] & 0xff);
        chk = chk & 0xff;
        chk = 255 - chk;

        byte chksum[] = new byte[1];
        chksum[0] = (byte) (chk & 0xff);
        js.write(chksum, 0, 1);
    }

    class ReaderThread extends Thread
    {
        public void run()
        {
            byte header[] = new byte[3];

            while (true) {

                // Do not read while in command mode (need 'OK' in that function)
                if (inCommandMode) {
                    TimeUtil.sleep(100);
                    continue;
                }
                int len = js.read(header, 0, 1);
                if (len != 1)
                    fatal();

                // wait for sync byte
                if (header[0] != 0x7e)
                    continue;

                // read length
                len = js.readFullyTimeout(header, 1, 2, INTRAPACKET_TIMEOUT_MS);
                if (len < 0)
                    fatal();
                if (len < 2)
                    continue;

                int length = ((header[1]&0xff)<<8) + (header[2]&0xff);

                byte payload[] = new byte[length];
                len = js.readFullyTimeout(payload, 0, payload.length, INTRAPACKET_TIMEOUT_MS);
                if (len < 0)
                    fatal();
                if (len < payload.length)
                    continue;

                byte chksum[] = new byte[1];
                len = js.readFullyTimeout(chksum, 0, 1, INTRAPACKET_TIMEOUT_MS);
                if (len < 0)
                    fatal();
                if (len < 1)
                    continue;

                /*
                   System.out.printf("RX buf: %d\n", length);
                   for (int i = 0; i < payload.length; i++)
                   System.out.printf("%02x ", payload[i]&0xff);
                   System.out.printf("\n");
                   */

                if (payload.length < 1)
                    continue;

                int api = payload[0]&0xff;

                // a TX Status message.
                if (api == 0x89 && payload.length == 3) {
                    int status = payload[2]&0xff;

                    if (status == 0) {
                        // successful transmission! Wake up the sender.
                        synchronized (txFrameNumberAckObject) {
                            txFrameNumberAck = payload[1]&0xff;
                            // wake up send thread.
                            txFrameNumberAckObject.notifyAll();
                        }
                    }

                    continue;
                }

                // a RX message
                if (api == 0x81 && payload.length >= START_IDX) {
                    int sourceAddr = ((payload[1]&0xff)<<8) + (payload[2]&0xff);
                    int rssi = payload[3] & 0xff;
                    int options = payload[4] & 0xff;

                    // Our header
                    int sourceID = payload[5];
                    int msgID = payload[6];

                    byte data[] = new byte[payload.length - START_IDX];

                    //reporter.reportRX(sourceID, msgID, data.length);

                    for (int i = 0; i < data.length; i++)
                        data[i] = payload[START_IDX + i];

                    // hand packet to radio listener
                    // note: could be a partial packet
                    for (RadioListener listener : listeners)
                        listener.packetReceived(XTendRadio.this, sourceAddr, data);
                }
            }
        }
    }

    /*class BandwidthReporter extends Thread
    {
        HashMap<Integer, xtend_report_t> received = new HashMap<Integer, xtend_report_t>();
        HashMap<Integer, Integer> lastReceived = new HashMap<Integer, Integer>();
        xtend_report_t sent = new xtend_report_t();

        public void run() {
            xtend_report_list_t report = new xtend_report_list_t();
            report.utime = TimeUtil.utime();

            sent.id = nodeID;
            while(true) {
                TimeUtil.sleep(1000); // Report everysecond
                int curHopPlan = hopPlan;
                HashMap<Integer, xtend_report_t> received = null;
                synchronized(this) {
                    long lastUtime = report.utime;
                    report.utime = TimeUtil.utime();
                    report.duration = (report.utime - lastUtime ) / 1E6;

                    report.sent = this.sent;
                    this.sent = new xtend_report_t();
                    this.sent.id = nodeID;

                    received = this.received;
                    this.received = new HashMap<Integer, xtend_report_t>();

                    report.nclients = received.size();
                    report.received = new xtend_report_t[report.nclients];
                    int i = 0;
                    for (xtend_report_t rep: received.values()) {
                        report.received[i] = rep;
                        i++;
                    }
                }


                lcm.publish("XTEND_REPORT_"+curHopPlan+"_"+nodeID, report);
            }
        }

        public synchronized void reportRX(int sourceID, int msgID, int len)
        {
            xtend_report_t report = received.get(sourceID);
            if (report == null) {
                report = new xtend_report_t();
                report.id = (byte)sourceID;
                received.put(sourceID, report);
            }

            Integer lastID = lastReceived.get(sourceID);

            if (lastID != null) {
                if (msgID < lastID) //rollover
                    report.lost_count += 128 + msgID - lastID - 1;
                else
                    report.lost_count += msgID - lastID - 1;
            }

            report.byte_count += len;
            report.packet_count++;

            lastReceived.put(sourceID, msgID);
        }

        public synchronized void reportTX(int msgID, int len)
        {
            Integer lastID = lastReceived.get((int)nodeID);

            if (lastID != null) {
                if (msgID < lastID) //rollover
                    sent.lost_count += 255 + msgID - lastID - 1;
                else
                    sent.lost_count += msgID - lastID - 1;

            }
            sent.byte_count += len;
            sent.packet_count++;

            lastReceived.put((int)nodeID, msgID);
        }

    }*/

    void fatal()
    {
        System.exit(-1);
    }


    public void addListener(RadioListener listener)
    {
        listeners.add(listener);
    }

    public static void main(String args[])
    {
        try {
            new XTendRadio(args[0], (byte)0, XTendRadio.BROADCAST, 0, FIRMWARE_US);
        } catch (IOException ex) {
            System.out.println("ex: "+ex);
        }
    }

}
