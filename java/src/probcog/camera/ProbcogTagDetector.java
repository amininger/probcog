package probcog.camera;

import java.awt.image.*;
import java.io.*;
import java.util.*;

import lcm.lcm.*;

import april.config.*;
import april.util.*;
import april.tag.*;

import magic2.lcmtypes.*;

public class ProbcogTagDetector implements LCMSubscriber
{
    LCM lcm = LCM.getSingleton();
    String imageChan;

    Config config;

    Object imageLock = new Object();
    image_t lastIm = null;

    public ProbcogTagDetector(GetOpt opts)
    {
        try {
            config = new ConfigFile(opts.getString("config"));
        } catch (IOException ex) {
            System.err.println("ERR: "+ex);
            ex.printStackTrace();
            System.exit(3);
        }

        imageChan = opts.getString("channel");
        lcm.subscribe(imageChan, this);

        new RunThread().start();
    }

    public void messageReceived(LCM lcm, String channel, LCMDataInputStream ins)
    {
        try {
            if (channel.equals(imageChan)) {
                synchronized (imageLock) {
                    lastIm = new image_t(ins);
                }
            }
        } catch (IOException ex) {
            System.err.println("ERR: Could not decode message on channel "+channel);
            ex.printStackTrace();
        }
    }

    public class RunThread extends Thread
    {
        TagFamily tf;
        TagDetector detector;

        public RunThread()
        {
            tf = new Tag36h11();
            tf.setErrorRecoveryBits(config.requireInt("tag_detection.tag.errorBits"));
            detector = new TagDetector(tf);

            detector.debug = false;
            detector.sigma = config.requireDouble("tag_detection.tag.sigma");
            detector.segSigma = config.requireDouble("tag_detection.tag.segSigma");
            detector.segDecimate = config.requireBoolean("tag_detection.tag.decimate");
            detector.minMag = config.requireDouble("tag_detection.tag.minMag");
            detector.maxEdgeCost = Math.toRadians(config.requireDouble("tag_detection.tag.maxEdgeCost_deg"));
            detector.magThresh = config.requireDouble("tag_detection.tag.magThresh");
            detector.thetaThresh = config.requireDouble("tag_detection.tag.thetaThresh");
            detector.WEIGHT_SCALE = config.requireInt("tag_detection.tag.weightScale");
        }

        public void run()
        {
            // Run as fast as possible
            while (true) {
                if (lastIm == null)
                    continue;

                BufferedImage im;
                synchronized (imageLock) {
                    im = debayerRGGB(lastIm.width, lastIm.height, lastIm.data);
                }

                // Process the image
                ArrayList<TagDetection> detections = detector.process(im, new double[] {im.getWidth()/2.0, im.getHeight()/2.0});

                // Publish detections
                tag_detection_list_t tagList = new tag_detection_list_t();
                tagList.utime = TimeUtil.utime();
                tagList.ndetections = detections.size();
                tagList.detections = new tag_detection_t[tagList.ndetections];
                for (int i = 0; i < tagList.ndetections; i++) {
                    TagDetection det = detections.get(i);

                    tag_detection_t td = new tag_detection_t();
                    td.tag_family_bit_width = 6;
                    td.tag_family_min_hamming_dist = 11;

                    td.id = det.id;
                    td.hamming_dist = (byte)det.hammingDistance;
                    td.goodness = 0.0f; // XXX

                    td.H[0][0] = (float) det.homography[0][0];
                    td.H[0][1] = (float) det.homography[0][1];
                    td.H[0][2] = (float) det.homography[0][2];
                    td.H[1][0] = (float) det.homography[1][0];
                    td.H[1][1] = (float) det.homography[1][1];
                    td.H[1][2] = (float) det.homography[1][2];
                    td.H[2][0] = (float) det.homography[2][0];
                    td.H[2][1] = (float) det.homography[2][1];
                    td.H[2][2] = (float) det.homography[2][2];

                    td.cxy[0] = (float) det.cxy[0];
                    td.cxy[1] = (float) det.cxy[1];

                    td.pxy[0][0] = (float) det.p[0][0];
                    td.pxy[0][1] = (float) det.p[0][1];
                    td.pxy[1][0] = (float) det.p[1][0];
                    td.pxy[1][1] = (float) det.p[1][1];
                    td.pxy[2][0] = (float) det.p[2][0];
                    td.pxy[2][1] = (float) det.p[2][1];
                    td.pxy[3][0] = (float) det.p[3][0];
                    td.pxy[3][1] = (float) det.p[3][1];

                    tagList.detections[i] = td;
                }

                lcm.publish("TAG_DETECTIONS_TX", tagList);
            }
        }
    }

    /** Returns an image of the same resolution as the input. **/
    public static BufferedImage debayerRGGB(int width, int height, byte d[])
    {
        BufferedImage im = new BufferedImage(width, height, BufferedImage.TYPE_INT_RGB);
        int out[] = ((DataBufferInt) (im.getRaster().getDataBuffer())).getData();

        // loop over each 2x2 bayer block and compute the pixel values for each element.
        for (int y = 0; y < height; y+=2) {
            for (int x = 0; x < width; x+=2) {

                int r = 0, g = 0, b = 0;

                // compute indices into bayer pattern for the nine 2x2 blocks we'll use.
                int X00 = (y-2)*width+(x-2);
                int X01 = (y-2)*width+(x+0);
                int X02 = (y-2)*width+(x+2);
                int X10 = (y+0)*width+(x-2);
                int X11 = (y+0)*width+(x+0);
                int X12 = (y+0)*width+(x+2);
                int X20 = (y+2)*width+(x-2);
                int X21 = (y+2)*width+(x+0);
                int X22 = (y+2)*width+(x+2);

                // handle the edges of the screen.
                if (y < 2) {
                    X00 += 2*width;
                    X01 += 2*width;
                    X02 += 2*width;
                }
                if (y+2 >= height) {
                    X20 -= 2*width;
                    X21 -= 2*width;
                    X22 -= 2*width;
                }
                if (x < 2) {
                    X00 += 2;
                    X10 += 2;
                    X20 += 2;
                }
                if (x+2 >= width) {
                    X02 -= 2;
                    X12 -= 2;
                    X22 -= 2;
                }

                // top left pixel (R)
                r = (d[X11]&0xff);
                g = ((d[X01+width]&0xff)+(d[X10+1]&0xff)+(d[X11+1]&0xff)+(d[X11+width]&0xff)) / 4;
                b = ((d[X00+width+1]&0xff)+(d[X10+width+1]&0xff)+(d[X10+width+1]&0xff)+(d[X11+width+1]&0xff)) / 4;
                out[y*width+x] = (r<<16)+(g<<8)+b;

                // top right pixel (G)
                r = ((d[X11]&0xff)+(d[X12]&0xff)) / 2;
                g = (d[X11+1]&0xff);
                b = ((d[X01+width+1]&0xff)+(d[X11+width+1]&0xff)) / 2;
                out[y*width+x+1] = (r<<16)+(g<<8)+b;

                // bottom left pixel (G)
                r = ((d[X11]&0xff)+(d[X21]&0xff)) / 2;
                g = (d[X11+width]&0xff);
                b = ((d[X10+width+1]&0xff)+(d[X11+width+1]&0xff)) / 2;
                out[y*width+width+x] = (r<<16)+(g<<8)+b;

                // bottom right pixel (B)
                r = ((d[X11]&0xff)+(d[X12]&0xff)+(d[X21]&0xff)+(d[X22]&0xff)) / 4;
                g = ((d[X11+1]&0xff)+(d[X11+width]&0xff)+(d[X12+width]&0xff)+(d[X21+1]&0xff))/ 4;
                b = (d[X11+width+1]&0xff);
                out[y*width+width+x+1] = (r<<16)+(g<<8)+b;
            }
        }

        return im;
    }


    public static void main(String[] args)
    {
        GetOpt gopt = new GetOpt();
        gopt.addBoolean('h', "help", false, "Show this help screen");
        gopt.addString('\0', "channel", "IMAGE", "image_t channel");
        gopt.addString('\0', "config", null, "Config w/ detector params");

        if (!gopt.parse(args) || gopt.getBoolean("help")) {
            gopt.doHelp();
            System.exit(1);
        }

        if (gopt.getString("config") == null) {
            System.err.println("ERR: Must specify a config file");
            System.exit(2);
        }

        new ProbcogTagDetector(gopt);
    }
}
