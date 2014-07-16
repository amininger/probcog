package probcog.robot.radio;

import java.io.*;

import april.util.*;

import probcog.util.*;

public class LCMXTend
{

    public static void main(String args[])
    {
        GetOpt opts  = new GetOpt();

        opts.addString('s',"hop-schedule","","Initial hoping plan [0-9]." +
                       "Default is robotID%4+1 (GCS can overide)");
        opts.addString('d',"device","/dev/radio","Default is /dev/radio ");
        opts.addString('f',"firmware","US","Pick either AU or US.");
        opts.addInt('i',"id",-1,"Defaults to invalid negative ID. Must be set!");
        opts.addBoolean('a',"dhsa",false,"Dynamic Hop Sequence Assignment");
        opts.addBoolean('h',"help",false,"See this help screen");

        if (!opts.parse(args))
	    {
            System.out.println("option error: "+opts.getReason());
	    }


        if (opts.getBoolean("help")) {
            System.out.println("Usage:");
            opts.doHelp();
            System.out.println("Dump:");
            opts.dump();
            System.exit(1);
        }

        System.out.println("LD_LIBRARY_PATH="+System.getProperty("LD_LIBRARY_PATH"));

        byte robotid =  (byte) opts.getInt("id");
        if (robotid < 0) {
            System.err.println("ERR: Must set non-negative ID");
            System.exit(-1);
        }

        int hopPlan = (robotid % 4) + 1;
        if (!opts.getString("hop-schedule").equals(""))
            hopPlan = Integer.parseInt(opts.getString("hop-schedule"));

        String device = opts.getString("device");

        int firmware = opts.getString("firmware").equals("US")? XTendRadio.FIRMWARE_US : XTendRadio.FIRMWARE_AU;

        try {
            new LCMRadio(new XTendRadio(device, robotid, 0xffff, hopPlan, firmware,
                                        opts.getBoolean("dhsa")),
                         robotid,
                         "_TX");
//                         RobotUtil.getConfig().requireString("radio.xtend.suffix"));
        } catch (IOException ex) {
            System.out.println("WRN: "+ex);
            System.exit(1);
        }
    }
}
