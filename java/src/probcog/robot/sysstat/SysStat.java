package probcog.robot.sysstat;

import lcm.lcm.*;

import april.util.*;
import april.config.*;
import probcog.lcmtypes.*;
import orc.*;
import april.jmat.*;

import java.util.*;
import java.io.*;
import java.net.*;

public class SysStat
{
    LCM lcm = LCM.getSingleton();
    Config config;
    GetOpt gopt;

    OrcThread orcThread[];
    int jiffies[] = new int[4]; // stats on jiffies from last iteration.

    // how many jiffies did the process use last time we polled it?
    HashMap<Integer, JiffieData> jiffieData = new HashMap<Integer, JiffieData>();

    // linux kernel parameter: number of ticks per second.
    static final int SC_CLK_TCK = 100;

    static class JiffieData
    {
        long mtime;
        int jiffies;
    }

    public SysStat(GetOpt gopt)
    {
        this.config = RobotUtil.getConfig();
        this.gopt = gopt;

        int norcs = gopt.getInt("norcs");
        orcThread = new OrcThread[norcs];
        for (int i = 0; i < norcs; i++) {
            String _ipaddr = gopt.getString("baseip");
            String toks[] = _ipaddr.split("\\.");
            String ipaddr = String.format("%s.%s.%s.%d", toks[0], toks[1], toks[2], Integer.parseInt(toks[3]) + i);
            orcThread[i] = new OrcThread(ipaddr);
            orcThread[i].start();
        }

        new RunThread().start();
    }

    class OrcThread extends Thread
    {
        String ipaddr;
        double dt;
        OrcStatus s;
        long utime;
        double version; // numerical orc version from vXX.YY on orc board

        OrcThread(String ipaddr)
        {
            this.ipaddr = ipaddr;
        }

        public void run()
        {
            try {
                Orc orc = new Orc(Inet4Address.getByName(ipaddr));
                // read version once only (does not change)

                String version = orc.getVersion();
                if (!version.startsWith("v")) {
                    System.out.println("WRN: Unrecognized firmware signature.");
                } else {
                    String toks[] = version.substring(1).split("[-]");
                    double numVersion = Integer.parseInt(toks[0]) +
                        Integer.parseInt(toks[1]) / 100.0;
                    System.out.println("NFO: numerical orc version: " + numVersion);
                    this.version = numVersion;
                }
                while (true) {
                    Tic tic = new Tic();
                    OrcStatus os = orc.getStatus();

                    synchronized(OrcThread.this) {
                        this.s = os;
                        this.dt = tic.toc();
                        this.utime = TimeUtil.utime();
                    }

                    TimeUtil.sleep(500);
                }

            } catch (Exception ex) {
            }
        }
    }

    class RunThread extends Thread
    {
        public void run()
        {
            while (true) {
                try {
                    doStats();
                } catch (Exception ex) {
                    System.out.println("WRN: "+ex);
                    ex.printStackTrace();
                }
                TimeUtil.sleep( (int) (1000 / gopt.getDouble("hz")));
            }
        }
    }

    void doStats()
    {
        system_status_t ss = new system_status_t();
        ss.utime = TimeUtil.utime();

        ///////////////////////////////////////////////////////////////////////
        // Bios Version
        try {
            String out = "";

            Process p = Runtime.getRuntime().exec("sudo dmidecode -s bios-version");

            BufferedReader ins = new BufferedReader(new InputStreamReader(p.getInputStream()));

            String line = null;
            while ((line = ins.readLine()) != null)
                out+=line;

            p.destroy();
            try {
                p.waitFor();
            } catch(InterruptedException e) {
            }

            ss.bios_version = Double.parseDouble(out.substring(out.indexOf("(")+1,
                                                               out.indexOf(")")));
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // Orc
        ss.norcs = (byte) gopt.getInt("norcs");
        ss.orc_status = new orc_status_t[ss.norcs];
        ss.orc_rtt_usec = new int[ss.norcs];

        for (int i = 0; i < ss.norcs; i++) {
            OrcThread ot = orcThread[i];
            synchronized(ot) {
                if (ot.s != null) {
                    ss.orc_status[i] = OrcStatusUtil.convert(ot.s, ot.version);
                    ss.orc_rtt_usec[i] = (int) (ot.dt * 1000000);

                    ss.robot_battery_v = 10.1 * ot.s.analogInputFiltered[11] / 65535.0 * 3.0;
                } else {
                    ss.orc_status[i] = new orc_status_t();
                    ss.orc_rtt_usec[i] = -99999;
                }
            }
        }

        ///////////////////////////////////////////////////////////////////////
        // CPU usage
        try {
            BufferedReader ins = new BufferedReader(new FileReader("/proc/stat"));
            String line;

            while ((line = ins.readLine()) != null) {
                String toks[] = line.split("\\s+");

                if (toks[0].equals("cpu")) {
                    int newjiffies[] = new int[4];
                    int djiffies[] = new int[4];
                    int total = 0;

                    for (int j = 0; j < 4; j++) {
                        newjiffies[j] = Integer.parseInt(toks[j+1]);
                        djiffies[j] = jiffies[j] - newjiffies[j];
                        total += djiffies[j];
                    }

                    ss.cpu_user = ((float) djiffies[0]) / total;
                    ss.cpu_lowpri = ((float) djiffies[1]) / total;
                    ss.cpu_system = ((float) djiffies[2]) / total;
                    ss.cpu_idle = ((float) djiffies[3]) / total;

                    jiffies = newjiffies;
                    break;
                }
            }

            ins.close();
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // uptime
        try {
            BufferedReader ins = new BufferedReader(new FileReader("/proc/uptime"));
            String line = ins.readLine();
            String toks[] = line.split("\\s+");
            ss.uptime = Double.parseDouble(toks[1]);
            ins.close();
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // loadavg
        try {
            BufferedReader ins = new BufferedReader(new FileReader("/proc/loadavg"));
            String line;

            while ((line = ins.readLine()) != null) {
                String toks[] = line.split("\\s+");
                if (toks.length < 3)
                    continue;
                for (int i = 0; i < 3; i++)
                    ss.loadavg[i] = Float.parseFloat(toks[i]);
            }
            ins.close();
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // Disk space
        if (true) {
            try {
                Process p = Runtime.getRuntime().exec("df "+gopt.getString("disk-space-path"));
                BufferedReader ins = new BufferedReader(new InputStreamReader(p.getInputStream()));
                ins.readLine(); // discard headers
                String line = ins.readLine();
                String toks[] = line.split("\\s+");
                ss.disk_space_mb = (int) (Long.parseLong(toks[3]) / 1024);
                p.destroy();
            } catch (Exception ex) {
                System.out.println("WRN: "+ex);
            }
        }

        ///////////////////////////////////////////////////////////////////////
        // Laptop Battery
        try {
            BufferedReader ins = new BufferedReader(new FileReader("/proc/acpi/battery/BAT0/state"));
            String line;

            while ((line = ins.readLine()) != null) {
                if (line.startsWith("remaining capacity:")) {
                    ss.laptop_battery_capacity_mah = findInt(line);
                }
                if (line.startsWith("present rate:")) {
                    ss.laptop_battery_rate_ma = findInt(line);
                }
            }
            ins.close();
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // Laptop Temperature
        try {
            BufferedReader ins = new BufferedReader(new FileReader("/proc/acpi/thermal_zone/THM0/temperature"));
            String line = ins.readLine();

            if (line != null)
                ss.laptop_temperature_deg_c = (float) findInt(line);

            ins.close();
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // Processor Throttling
        try {
            // get number of processors
            BufferedReader in = new BufferedReader(new FileReader("/sys/devices/system/cpu/online"));
            String line = in.readLine();
            int start = 0;
            int end = 1;

            if (line != null) {
                start   = Integer.parseInt(line.substring(0, line.indexOf("-")));
                end     = Integer.parseInt(line.substring(line.indexOf("-")+1));
            }

            int ncores = end - start + 1;
            in.close();

            ////////////////////////////////////////
            boolean throttled = false;

            ss.ncores = (byte) ncores;
            ss.cpuinfo = new system_cpuinfo_t[ncores];
            // for each processor...
            for (int i=start; i <= end; i++) {
                int idx = i - start;
                system_cpuinfo_t info  = getCPUInfo(i);
                ss.cpuinfo[idx] = info;
            }
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // Memory info
        try {
            BufferedReader ins = new BufferedReader(new FileReader("/proc/meminfo"));
            String line;

            while ((line = ins.readLine()) != null) {
                if (line.startsWith("MemFree:")) {
                    ss.mem_free = findInt(line);
                }
                if (line.startsWith("Buffers:")) {
                    ss.mem_buffers = findInt(line);
                }
                if (line.startsWith("Cached:")) {
                    ss.mem_cached = findInt(line);
                }

            }
            ins.close();
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }

        ///////////////////////////////////////////////////////////////////////
        // Piggish processes
        if (true) {

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

                        JiffieData jd = jiffieData.get(pid);

                        if (jd != null) {
                            ProcData pd = new ProcData();
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

            Collections.sort(data, new ProcDataMemComparator());
            for (int i = 0; i < Math.min(10, data.size()); i++)
                data.get(i).selected = true;

            Collections.sort(data, new ProcDataCPUComparator());
            for (int i = 0; i < Math.min(10, data.size()); i++)
                data.get(i).selected = true;


            ArrayList<ProcData> selectedData = new ArrayList<ProcData>();
            for (ProcData pd : data)
                if (pd.selected)
                    selectedData.add(pd);
            data = selectedData;

            int nprocs = Math.min(10, data.size());

            ss.nprocs = nprocs;
            ss.processes = new system_process_data_t[nprocs];

            for (int i = 0; i < nprocs; i++) {
                ProcData pd = data.get(i);
                ss.processes[i] = new system_process_data_t();
                ss.processes[i].cmd = pd.cmd;
                ss.processes[i].mem = (float) pd.mem;
                ss.processes[i].cpu = (float) pd.cpu;
            }
        }

        ///////////////////////////////////////////////////////////////////////
        // publish!
        ss.other = "";

        lcm.publish("SYSTEM_STATUS", ss);
    }

    static class ProcDataCPUComparator implements Comparator<ProcData>
    {
        public int compare(ProcData a, ProcData b)
        {
            return -Double.compare(a.cpu, b.cpu);
        }
    }

    static class ProcDataMemComparator implements Comparator<ProcData>
    {
        public int compare(ProcData a, ProcData b)
        {
            return -Double.compare(a.mem, b.mem);
        }
    }

    static class ProcData
    {
        boolean selected;

        double cpu, mem;
        String cmd;
    }

    static int findInt(String s)
    {
        String toks[] = s.split("\\s+");
        for (int i = 0; i < toks.length; i++) {
            try {
                return Integer.parseInt(toks[i]);
            } catch (Exception ex) {
            }
        }
        return 0;
    }

    static double findDouble(String s)
    {
        String toks[] = s.split("\\s+");
        for (int i = 0; i < toks.length; i++) {
            try {
                return Double.parseDouble(toks[i]);
            } catch (Exception ex) {
            }
        }
        return 0;
    }

    public system_cpuinfo_t getCPUInfo(int cpu)
    {
        system_cpuinfo_t info = new system_cpuinfo_t();
        String basepath = "/sys/devices/system/cpu/cpu"+cpu+"/";
        boolean failed = false;
        int maxFreq = 0;

        // thermal throttle count
        try {
            BufferedReader in = new BufferedReader(new FileReader(basepath + "thermal_throttle/throttle_count"));
            info.thermal_throttle_count = (short) Integer.parseInt(in.readLine());
            in.close();
        } catch (IOException ex) {
            System.out.println("WRN: Failed to load file. Ex: "+ex);
            failed = true;
        }

        // throttle details
        {
            info.performance_aut = new byte[3];
            info.throttling_aut  = new byte[3];
            int active_performance  = -1,   active_throttle     = -1;
            int user_performance    = -1,   user_throttle       = -1;
            int thermal_performance = -1,   thermal_throttle    = -1;

            try {
                BufferedReader insl = new BufferedReader(new FileReader("/proc/acpi/processor/CPU" + cpu + "/limit"));

                String linel;
                while ((linel = insl.readLine()) != null ) {
                    if (linel.startsWith("active")) {
                        active_performance  = Integer.parseInt(Character.toString(linel.charAt(linel.indexOf("P")+1)));
                        active_throttle     = Integer.parseInt(Character.toString(linel.charAt(linel.indexOf("T")+1)));
                    }
                    if (linel.startsWith("user")) {
                        user_performance    = Integer.parseInt(Character.toString(linel.charAt(linel.indexOf("P")+1)));
                        user_throttle       = Integer.parseInt(Character.toString(linel.charAt(linel.indexOf("T")+1)));
                    }
                    if (linel.startsWith("thermal")) {
                        thermal_performance = Integer.parseInt(Character.toString(linel.charAt(linel.indexOf("P")+1)));
                        thermal_throttle    = Integer.parseInt(Character.toString(linel.charAt(linel.indexOf("T")+1)));
                    }
                }

                insl.close();
            } catch (IOException ex) {
                System.out.println("WRN: "+ex);
            }

            info.performance_aut[0] = (byte) active_performance;
            info.performance_aut[1] = (byte) user_performance;
            info.performance_aut[2] = (byte) thermal_performance;
            info.throttling_aut[0]  = (byte) active_throttle;
            info.throttling_aut[1]  = (byte) user_throttle;
            info.throttling_aut[2]  = (byte) thermal_throttle;
        }

        // cpufreq directory existence
        {
            File f = new File(basepath+"cpufreq/");
            if (f.exists())
                info.cpufreq_enabled = true;
            else
                info.cpufreq_enabled = false;
        }

        // frequency from /proc/cpuinfo
        {
            try {
                BufferedReader in = new BufferedReader(new FileReader("/proc/cpuinfo"));

                String line;

                // find processor
                while ((line = in.readLine()) != null) {
                    if (line.startsWith("processor")) {
                        int proc = findInt(line);
                        if (proc == cpu)
                            break;
                    }
                }

                // find frequency for given processor
                while ((line = in.readLine()) != null) {
                    if (line.startsWith("cpu MHz")) {
                        double freq = findDouble(line);
                        info.frequency_mhz = (short) freq;
                        break;
                    }
                }

                in.close();
            } catch (IOException ex) {
                System.out.println("WRN: "+ex);
            }
        }
        return info;
    }

    public static void main(String args[])
    {
        GetOpt gopt = new GetOpt();
        gopt.addDouble('r', "hz", .5, "Rate to publish");
        gopt.addBoolean('h', "help", false, "Show help");
        gopt.addInt('\0', "norcs", 0, "Number of OrcBoards to query");
        gopt.addString('\0', "baseip", "192.168.237.7", "IP address of first orc");
        gopt.addString('\0', "disk-space-path", "/var/tmp", "Path to monitor free space");

        gopt.parse(args);
        if (gopt.getBoolean("help")) {
            gopt.doHelp();
            return;
        }

        new SysStat(gopt);
    }
}
