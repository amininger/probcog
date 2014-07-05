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
    static TagDetectionGenerator tagGenerator;
    ArrayList<Listener> listeners = new ArrayList<Listener>();

    public interface Listener
    {
        public void handleImage(LazyImage lim);
    }

    public CameraDriver(Config config, String channel) throws IOException
    {
        this.config = config;
        this.channel = channel;

        if (channel.isEmpty()) {
            isrc = ImageSource.make(config.requireString("url"));
            makeSyncDetector();
            new ReaderThread().start();
        }
        else {
            System.out.println("Subscribing to LCM channel '"+this.channel+"'");
            lcm.subscribe(channel, this);
            new IdleThread().start();
        }
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

            // long utime = TimeUtil.utime();
            // long dt = utime - lastImgAnalysis_us;
            // int curFollowID = followID;
            tagGenerator.addImage(lim);

            // if (((curFollowID == -1) && dt > FLIP_BOOK_IMAGE_PERIOD_US) ||
            //     ((curFollowID != -1) && dt > FOLLOW_IMAGE_PERIOD_US)){
            //     /*comment_t comm = new comment_t();
            //       comm.utime = TimeUtil.utime();
            //       comm.message = "no-op";
            //       lcm.publish("LOOK_FOR_TAGS_RATE", comm);*/
            //     // Notify tag detector

            //     lastImgAnalysis_us = utime;
            // }


            // for (Listener listener : listeners)
            //     listener.handleImage(lim);
        }
    }

    /** Convenience method if sync detector must be remade when toggling isrc format.
     **/
    public void makeSyncDetector()
    {
        if (config == null)
            return;

        syncDetector = new SyncErrorDetector(config.getChild("sync"));
    }


    class ReaderThread extends Thread
    {
        public void toggleImageSourceFormat(ImageSource isrc)
        {
            isrc.stop();
            int currentFormat = isrc.getCurrentFormatIndex();
            isrc.setFormat("GRAY8");
            isrc.setFormat(currentFormat);
            isrc.start();
        }

        public void run()
        {
            try {
                readFrames();
            }
            catch (Exception e) {
                System.out.println("ERR: CAM: " + e);
            }
            finally {
                // If we exited out of the driver loop its a serious error. So
                // exit and let ProcMan restart us cleanly
                System.exit(-1);
            }
        }

        private void readFrames()
        {
            isrc.start();

            while (true) {
                FrameData frmd = isrc.getFrame();
                if (frmd == null) {
                    // XXX Right thing to do?
                    System.exit(-1);
                }

                // Check sync status
                int syncResult = SyncErrorDetector.SYNC_GOOD;
                if (syncDetector != null) {
                    syncDetector.addTimePointGreyFrame(frmd.data);

                    syncResult = syncDetector.verify();
                }

                // Take serious action
                if (syncResult == SyncErrorDetector.RECOMMEND_ACTION) {
                    // Toggle image source format to clear frame errors (generally works, but not always)
                    System.out.println("WRN: CAM: libdc sync did not recover in allotted time." +
                                       " Toggling isrc format and clearing detector");
                    toggleImageSourceFormat(isrc);
                    makeSyncDetector();

                    // We need to reset lastLim so we don't publish an old frame
                    lastLim = null;

                    continue;
                }

                // Skip processing if the sync is bad
                if (syncResult == SyncErrorDetector.SYNC_BAD) {
                    // We need to reset lastLim so we don't publish an old frame
                    lastLim = null;
                    continue;
                }

                // Sync is good, we can use this frame
                LazyImage lim = new LazyImage(frmd.data, frmd.ifmt, frmd.utime);

                // Call the listeners with the previous frame
                // NOTE: We add a 1-frame delay to the pipe to ensure that the frame wasn't half-bad
                // and followed by a detectably-bad frame (which is a common issue)
                // Of course, we can only publish if this isn't the first good frame

                if (lastLim != null)
                    for (Listener listener : listeners)
                        listener.handleImage(lastLim);

                lastLim = lim;
            }
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
        System.out.println("Starting camera driver");

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
        tagGenerator = new TagDetectionGenerator(servos);

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
