package probcog.slam;

import java.awt.image.*;
import java.io.*;
import javax.imageio.*;
import java.util.*;

import april.camera.*;
import april.camera.models.*;
import april.camera.calibrator.*;
import april.config.*;
import april.util.*;
import april.tag.*;

import probcog.camera.ProbcogTagDetector;

import magic2.lcmtypes.*;

public class TagFix
{
    Config config;
    View input, output;
    Rasterizer rasterizer;

    // Pass in a camera calibration config.
    public TagFix(Config config) throws IOException
    {
        this.config = config;

        // Create the input view
        String classname = config.requireString("class");
        //Object obj = ReflectUtil.createObject(classname, this.config);
        Object obj = new AngularPolynomialCalibration(config);
        assert (obj != null);
        assert (obj instanceof Calibration);
        input = (Calibration) obj;

        // Create the output view
        output = new MaxRectifiedView(input);

        // Create the rasterizer
        rasterizer = new NearestNeighborRasterizer(input, output);
    }

    boolean flip = false;
    public BufferedImage getImage(image_t it)
    {
        // 1) convert image to BufferedImage
        BufferedImage im = ProbcogTagDetector.debayerRGGB(it.width, it.height, it.data);

        // 2) Undistort the image
        BufferedImage r_im = rasterizer.rectifyImage(im);

        // XXX For debugging: save to file
        /*if (true) {
            String name0 = "/tmp/tag-image-0.png";
            String name1 = "/tmp/tag-image-1.png";
            try {
                File f;
                if (flip) {
                    f = new File(name1);
                } else {
                    f = new File(name0);
                }
                flip = !flip;

                ImageIO.write(r_im, "PNG", f);
            } catch (IOException ex) {
                ex.printStackTrace();
            }
        }*/

        return r_im;
    }

    public tag_detection_list_t getTags(BufferedImage im, TagDetector d)
    {
        // 3 ) Find the tags
        tag_detection_list_t tdl = ProbcogTagDetector.getDetections(im, d);

        return tdl;
    }

    public double[] getCameraParams()
    {
        double[][] K = output.copyIntrinsics();
        return new double[] {K[0][0], K[1][1], K[0][2], K[1][2]};
    }
}
