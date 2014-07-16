package probcog.robot.radio;

import java.awt.*;
import javax.swing.*;

import java.io.*;
import lcm.lcm.*;

import april.lcmtypes.*;
import april.util.*;

// Debugging class to spoof packets of specific sizes
public class PacketTester
{

    public static void main(String args[])
    {
        GetOpt opts = new GetOpt();
        opts.addBoolean('h',"help",false,"Show this help screen");
        opts.addInt('n',"bytes",1950,"Number of bytes in image");
        opts.addInt('\0',"images",10,"Number of images to send");

        if (!opts.parse(args) || opts.getBoolean("help")) {
            opts.doHelp();
            System.exit(0);
        }

        LCM lcm = LCM.getSingleton();

        int n = opts.getInt("bytes");

        for (int i = 0; i < opts.getInt("images"); i++) {

            image_t img = new image_t();
            img.utime = TimeUtil.utime();
            img.height = 1;
            img.width  = (short)n;
            img.pixelformat = 1;
            img.size = n;
            img.image = new byte[n];

            int size =0;
            try{
                LCMDataOutputStream dout =new LCMDataOutputStream();
                img.encode(dout);
                size = dout.size();
            }catch(IOException e){}
            lcm.publish("BOGUS_IMAGE_TX", img);
            System.out.println("Sent new message of size " + size);

            TimeUtil.sleep(1000);
        }
        System.out.println("Done.");
        System.exit(0);
    }

}
