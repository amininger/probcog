package probcog.robot.sysstat;

import lcm.lcm.*;

import april.config.*;
import april.util.*;

import probcog.lcmtypes.*;

import java.io.*;
import java.util.*;

public class AlarmMonitor implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();
    Config config;
    double firmware_rev;
    double orcVersion;

    AlarmUtil au;

    HashMap<String, ChannelStats> channelStats = new HashMap<String, ChannelStats>();

    robot_command_list_t cmdList;

    long startmtime = System.currentTimeMillis();
    orc_status_t last_orc_status[];
    TimeSync gcSync = new TimeSync(1E6, 0, 0.001, 0.5);

    int estopMismatchCount = 0;

    boolean ensureMonotonicMsgs;  // only accept newer msge (no loops in log playback)

    static class ChannelStats
    {
        String channel;
        int    count;
    }

    public AlarmMonitor(Config config)
    {
        this.ensureMonotonicMsgs = RobotUtil.getConfig().getBoolean("lcm.accept_only_monotonic_utimes", true);

        this.config = config;
        this.firmware_rev = config.requireDouble("BIOS_revision");
        this.orcVersion = config.requireDouble("orc_version");

        au = new AlarmUtil(config);

        lcm.subscribe(".*", this);

        new RateMonitor().start();
        new CmdListMonitor().start();

        System.out.println("NFO: Initialized.");
    }

    class RateMonitor extends Thread
    {
        public void run()
        {
            Config rateConfig = config.getChild("lcm_rates");

            while (true) {
                long utime0 = TimeUtil.utime();
                TimeUtil.sleep((int) (config.requireDouble("lcm_rates_period")*1000));
                long utime1 = TimeUtil.utime();

                double dt = (utime1 - utime0) / 1000000.0;

                synchronized(AlarmMonitor.this) {
                    for (String key : rateConfig.getKeys()) {
                        ChannelStats cs = channelStats.get(key);
                        if (cs == null) {
                            au.alarm("No messages on "+key);
                            continue;
                        }

                        double hz = cs.count / dt;
                        double minmax[] = rateConfig.getDoubles(key);

//                        System.out.println("rate "+key+" "+hz);

                        if (hz < minmax[0])
                            au.alarm("Low message rate on "+key+", "+((int) hz)+" hertz");
                        if (hz > minmax[1])
                            au.alarm("High message rate on "+key+", "+((int) hz)+" hertz");
                    }

                    // start over.
                    for (ChannelStats cs : channelStats.values())
                        cs.count = 0;
                }
            }
        }
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        // update message statistics
        synchronized(this) {
            ChannelStats stats = channelStats.get(channel);
            if (stats == null) {
                stats = new ChannelStats();
                stats.channel = channel;
                channelStats.put(channel, stats);
            }
            stats.count++;
        }

        // handle specific messages
        try {
            if (channel.equals("CMDS_"+RobotUtil.getGNDID())) {
                robot_command_list_t _cmdList = new robot_command_list_t(ins);
                if (cmdList == null || !ensureMonotonicMsgs || cmdList.utime < _cmdList.utime) {
                    cmdList = _cmdList;
                    gcSync.update(TimeUtil.utime(), cmdList.utime);
                }
            }

            if (channel.equals("SYSTEM_STATUS")) {
                system_status_t msg = new system_status_t(ins);

                if (msg.laptop_temperature_deg_c >= config.requireDouble("laptopTemp"))
                    au.alarm("Laptop temperature "+((int) msg.laptop_temperature_deg_c));

                if (msg.robot_battery_v <= config.requireDouble("batteryVoltage"))
                    au.alarm("Battery voltage "+((int) msg.robot_battery_v));

                if (msg.norcs != 2)
                    au.alarm("Missing Orc board");

                if (msg.bios_version < firmware_rev)
                    au.alarm("BIOS version out of date, revision "+msg.bios_version);

                boolean anyCPUFreq = false;
                for (system_cpuinfo_t info : msg.cpuinfo)
                    anyCPUFreq |= info.cpufreq_enabled;
                if (anyCPUFreq)
                    au.alarm("C P U throttling is enabled");

                long now = TimeUtil.utime();

                // do the orc boards disagree about the ESTOP
                // switch? We only report error if this happens a
                // couple times in a row, since there's
                // intrinsically a race condition here.
                if ((msg.orc_status[0].status_flags & 1) != (msg.orc_status[1].status_flags & 1)) {
                    estopMismatchCount++;
                    if (estopMismatchCount > 2)
                        au.alarm("Orc E Stop mismatch");
                } else {
                    estopMismatchCount = 0;
                }

                for (int i = 0; i < msg.norcs; i++) {
                    double dt = (now - msg.orc_status[i].utime) / 1000000.0;
                    if (dt > config.requireDouble("orcTimeout"))
                        au.alarm("Orc " + i + " timeout");
                    else
                        if (msg.orc_rtt_usec[i] / 1000000.0 > config.requireDouble("orcRTT"))
                            au.alarm("Orc " + i + " RTT");

                    // fault if value is zero.
                    boolean motfault0 = (msg.orc_status[i].simple_digital_values & (1<<8)) == 0;
                    boolean motfault1 = (msg.orc_status[i].simple_digital_values & (1<<10)) == 0;
                    boolean motfault2 = (msg.orc_status[i].simple_digital_values & (1<<12)) == 0;

                    if (motfault0 || motfault1) {
                        String str;
                        if (motfault0 && motfault1)
                            str = "s 0 and 1";
                        else if (motfault0)
                            str = " 0";
                        else
                            str = " 1";
                        au.alarm("Orc " + i + " motor fault" + str);
                    }
                    if (last_orc_status != null && last_orc_status.length > i) {
                        if (msg.orc_status[i].utime_orc < last_orc_status[i].utime_orc)
                            au.alarm("Orc "+i+" reset. Repeat, Orc "+i+" reset");
                    }
                    if (msg.orc_status[i].version < orcVersion)
                        au.alarm("Orc "+i+" version out of date");
                }

                last_orc_status = msg.orc_status;

                if (msg.mem_free + msg.mem_cached < config.requireDouble("freeMemory"))
                    au.alarm("Low memory");
            }
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
        }
    }

    // generate an alarm when CMDS_ expires.
    class CmdListMonitor extends Thread
    {
        public void run()
        {
            boolean haveValidLast = false;
            boolean warnedBadSync = false;

            while (true) {
                TimeUtil.sleep(500);

                boolean haveValidNow = false;
                if (cmdList != null) {
                    double age = (TimeUtil.utime() - gcSync.getHostUtime(cmdList.utime)) / 1000000.0;
                    if (age < config.getRoot().requireDouble("robot.driver.cmd_timeout")) {
                        haveValidNow = true;
                    } else {
                        au.alarm("NOTICE", "Bad sink with ground control");
                        warnedBadSync = true;
                    }
                }

                if (haveValidNow && !haveValidLast)
                    au.alarm("NOTICE", "Sinked with ground control");

                if (!haveValidNow && haveValidLast)
                    au.alarm("NOTICE", "Ground control command has expired");

                haveValidLast = haveValidNow;
            }
        }
    }

    public static void main(String args[])
    {
        Config config = RobotUtil.getConfig();
        new AlarmMonitor(config.getChild("alarm"));

        while (true) {
            TimeUtil.sleep(100);
        }
    }
}
