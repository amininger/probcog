package probcog.camera;

import java.awt.image.*;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.image.*;
import april.jcam.*;
import april.lcm.*;
import april.lcmtypes.*;
import april.util.*;

import probcog.servo.*;
import probcog.util.*;

public class CameraDriver implements LCMSubscriber
{
    Config config;
    LCM lcm = LCM.getSingleton();
    String channel;

    ImageSource isrc;
    CameraDriver.LazyImage lastLim;

    ArrayList<Listener> listeners = new ArrayList<Listener>();

    public interface Listener
    {
        public void handleImage(LazyImage lim);
    }

    public CameraDriver(Config config, String channel) throws IOException
    {
        this.config = config;
        this.channel = channel;

        System.out.println("Subscribing to LCM channel '"+this.channel+"'");
        lcm.subscribe(channel, this);

        new IdleThread().start();
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            messageReceivedEx(channel, ins);
        }
        catch (IOException ex) {
            System.out.println("Exception: " + ex);
        }
    }

    void messageReceivedEx(String channel, LCMDataInputStream ins) throws IOException
    {
        if (channel.equals(this.channel)) {
            image_t imt = new image_t(ins);
            BufferedImage im = image_t_util.decode(imt);
            im = ImageUtil.convertImage(im, BufferedImage.TYPE_INT_RGB);

            LazyImage lim = new LazyImage(im, imt.utime);
            for (Listener listener : listeners)
                listener.handleImage(lim);
        }
    }

    public static class LazyImage
    {
        public  final long utime;
        private final byte buf[];
        private final ImageSourceFormat ifmt;

        private volatile BufferedImage im;


        LazyImage(byte buf[], ImageSourceFormat ifmt, long utime)
        {
            this.utime = utime;
            this.buf = buf;
            this.ifmt = ifmt;
        }

        LazyImage(BufferedImage im, long utime)
        {
            this.im = im;

            this.utime = utime;
            this.buf = null;

            ImageSourceFormat ifmt = new ImageSourceFormat();
            ifmt.width  = im.getWidth();
            ifmt.height = im.getHeight();
            this.ifmt   = ifmt;
        }

        public synchronized BufferedImage getImage()
        {
            // Double checked locking. Note that the assigned
            // variable is volatile
            if (im == null) {
                synchronized (this) {
                    if (im == null) {
                        im = ImageConvert.convertToImage(ifmt.format, ifmt.width,
                                ifmt.height, buf);
                    }
                }
            }

            return im;
        }

        public int getWidth()
        {
            return ifmt.width;
        }

        public int getHeight()
        {
            return ifmt.height;
        }
    }

    public void addListener(Listener listener)
    {
        listeners.add(listener);
    }

    class IdleThread extends Thread
    {
        public void run()
        {
            while (true) {
                TimeUtil.sleep(300);
            }
        }
    }

    public static void main(String args[])
    {
        GetOpt opts  = new GetOpt();

        opts.addBoolean('h',"help",false,"See this help screen");
        opts.addString('c',"channel","","Subscribe to images on specified LCM Channel (do not open camera)");

        if (!opts.parse(args))
	    {
            System.out.println("option error: "+opts.getReason());
	    }

        if (opts.getBoolean("help")){
            System.out.println("Usage:");
            opts.doHelp();
            System.exit(1);
        }

        String channel = opts.getString("channel");

        PanTiltServoController servos = new PanTiltServoController();
        TagDetectionGenerator tagGenerator = new TagDetectionGenerator(servos);

        // CameraTracker camTracker = new CameraTracker(servos);

        try {
            System.out.println(channel);
            CameraDriver cd = new CameraDriver(Util.getConfig().getChild("camera"), channel);

            // cd.addListener(camTracker);
        } catch (IOException ex) {
            System.err.println("ERR: main(): "+ex);
            System.exit(1);
        }
    }
}
