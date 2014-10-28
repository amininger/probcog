package probcog.slam;

import java.awt.image.*;
import java.io.*;
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

    public BufferedImage getImage(image_t it)
    {
        // 1) convert image to BufferedImage
        BufferedImage im = ProbcogTagDetector.debayerRGGB(it.width, it.height, it.data);

        // 2) Undistort the image
        BufferedImage r_im = rasterizer.rectifyImage(im);

        return r_im;
    }

    public tag_detection_list_t getTags(BufferedImage im, TagDetector d)
    {
        // 3 ) Find the tags
        tag_detection_list_t tdl = ProbcogTagDetector.getDetections(im, d);

        return tdl;
    }
}
