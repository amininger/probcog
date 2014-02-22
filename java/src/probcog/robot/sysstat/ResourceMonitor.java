package magic.sysstat;

import java.util.*;
import java.io.*;
import java.net.*;

import lcm.lcm.*;
import april.lcmtypes.*;
import magic.lcmtypes.*;

import april.util.*;
import april.config.*;

import magic.util.*;

public class ResourceMonitor implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();
    Config config;
    double rate_hz = 0.5;

    procman_process_list_t  ppl;
    procman_status_list_t   psl;

    // how many jiffies did the process use last time we polled it?
    HashMap<Integer, JiffieData> jiffieData = new HashMap<Integer, JiffieData>();

    // linux kernel parameter: number of ticks per second.
    static final int SC_CLK_TCK = 100;

    static class JiffieData
    {
        long mtime;
        int jiffies;
    }

    public ResourceMonitor()
    {
        this.config = RobotUtil.getConfig();

        rate_hz = this.config.getDouble("resourceMon.rate_hz", rate_hz);

        lcm.subscribe("PROCMAN_PROCESS_LIST", this);
        lcm.subscribe("PROCMAN_STATUS_LIST", this);
        new RunThread().start();
    }

    class RunThread extends Thread
    {
        public void run()
        {
            while (true) {
                try {
                    doResourceStatus();
                } catch (Exception e) {
                    System.out.println("WRN: "+e);
                    e.printStackTrace();
                }

                TimeUtil.sleep( (int) (1000 / rate_hz) );
            }
        }
    }

    public synchronized void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals("PROCMAN_PROCESS_LIST"))
                ppl = new procman_process_list_t(ins);
            if (channel.equals("PROCMAN_STATUS_LIST"))
                psl = new procman_status_list_t(ins);
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }

    static class ProcData
    {
        boolean selected;

        int pid;
        double cpu, mem;
        String cmd;
    }

    public synchronized void doResourceStatus()
    {
        if (ppl == null || psl == null)
            return;

        File proc = new File("/proc");
        File ents[] = proc.listFiles();

        ArrayList<ProcData> data = new ArrayList<ProcData>();

        for (File e : ents) {
            try {
                String s = e.getName();
                if (Character.isDigit(s.charAt(0))) {
                    int pid = Integer.parseInt(s);

                    BufferedReader ins = new BufferedReader(new FileReader("/proc/"+pid+"/stat"));
                    String line = ins.readLine();
                    String toks[] = line.split("\\s+");
                    int jiffies = Integer.parseInt(toks[13]) + Integer.parseInt(toks[14]);
                    int rss = Integer.parseInt(toks[23]);
                    ins.close();

                    long mtime = System.currentTimeMillis();

                    ins = new BufferedReader(new FileReader("/proc/"+pid+"/cmdline"));
                    String cmd = ins.readLine();
                    ins.close();

                    if (cmd == null)
                        continue;

                    cmd = cmd.replace("\0", " ");
                    cmd = cmd.trim();

                    JiffieData jd = jiffieData.get(pid);

                    if (jd != null) {
                        ProcData pd = new ProcData();
                        pd.pid = pid;
                        pd.cmd = cmd;
                        pd.mem = rss;
                        double dms = mtime - jd.mtime;
                        pd.cpu = (1000.0 / dms) * (jiffies - (int) jd.jiffies) / SC_CLK_TCK;
                        data.add(pd);
                    } else {
                        jd = new JiffieData();
                        jiffieData.put(pid, jd);
                    }

                    jd.jiffies = jiffies;
                    jd.mtime = mtime;
                }
            } catch (Exception ex) {
                // the process file can disappear out from under us. No biggie.
            }
        }

        // init publishable lcmtype
        resource_monitor_t moni = new resource_monitor_t();
        moni.utime              = TimeUtil.utime();
        moni.nprocs             = psl.nprocs;
        moni.statuses           = new process_status_t[moni.nprocs];
        for (int i=0; i < moni.nprocs; i++) {
            moni.statuses[i] = new process_status_t();
            moni.statuses[i].procid     = -1;
            moni.statuses[i].pid        = -1;
            moni.statuses[i].running    = false;
            moni.statuses[i].data       = new system_process_data_t();
            moni.statuses[i].data.cmd   = "";
        }

        // fill in processes if we can find them
        for (ProcData pd : data) {
            //System.out.printf("Trying to match '%s'\n", pd.cmd);
            for (procman_status_t ps : psl.statuses) {
                // find corresponding procman_process_t
                procman_process_t pp = new procman_process_t();
                for (procman_process_t ppt : ppl.processes) {
                    if (ppt.procid == ps.procid)
                        pp = ppt;
                }
                assert(pp != null);

                // are pd and ps referring to the same process?
                if (pd.cmd.equals(pp.cmdline)) {
                    //System.out.printf("\tmatch!");
                    process_status_t pstatus = new process_status_t();

                    pstatus.procid  = ps.procid;
                    pstatus.pid     = pd.pid;
                    pstatus.running = ps.running;

                    system_process_data_t pdata = new system_process_data_t();
                    pdata.cmd = pd.cmd;
                    pdata.cpu = (float) pd.cpu;
                    pdata.mem = (float) pd.mem;

                    pstatus.data = pdata;

                    moni.statuses[pstatus.procid-1] = pstatus;
                } else {
                    //System.out.printf("\t      ");
                }

                //System.out.printf("\tcandidate: '%s'\n", pp.cmdline);
            }
        }

        lcm.publish("RESOURCE_MONITOR", moni);
    }

    public static void main(String args[])
    {
        new ResourceMonitor();
    }
}

