package probcog.util;

import april.tag.TagDetection;
import april.tag.CameraUtil;
import april.jmat.LinAlg;
import april.lcmtypes.pose_t;

import probcog.lcmtypes.*;

public class TagUtil
{
        // camera calibration
    static final double camera_fc[]  = Util.getConfig().requireDoubles("cameraCalibration.intrinsics.fc");
    static final double camera_cc[]  = Util.getConfig().requireDoubles("cameraCalibration.intrinsics.cc");
    static final double camera_kc[]  = Util.getConfig().requireDoubles("cameraCalibration.intrinsics.kc");
    static final double camera_skew= Util.getConfig().requireDouble("cameraCalibration.intrinsics.skew");
    static final double imHeight = Util.getConfig().requireDouble("cameraCalibration.imHeight");
    static final double imWidth = Util.getConfig().requireDouble("cameraCalibration.imWidth");

    public static double[][] getTagToPose(TagDetection d, double tagSize_m)
    {
        // get center pixel for tag
        //double imHeight = 480;
        //double imWidth = 743;

        double M[][] = CameraUtil.homographyToPose(-camera_fc[0], camera_fc[1], imWidth/2, imHeight/2, d.homography);
        M = CameraUtil.scalePose(M, 2.0, tagSize_m);

        // Hardcoded transform for vertical facing camera
        double[][] xform = new double[][]{{ 0,-1, 0, 0},
                                          {-1, 0, 0, 0},
                                          { 0, 0,-1, 0},
                                          { 0, 0, 0, 1}};


        // Rotate around Y to correct for upside down tags.
        M = LinAlg.matrixAB(xform, M);
        M = LinAlg.matrixAB(M, LinAlg.rotateZ(Math.PI));

        return M;
    }

    public static double[] getTagXyzrpy(double O2B[][], double Observer[][], boolean transformToRobotPose)
    {
        double O2W[][] = LinAlg.multiplyMany(Observer, O2B);

        // for the given face we're viewing, convert from tag frame to robot frame
        double faceTransform[][] = (transformToRobotPose ?
                                    getFaceTransformRobot(LinAlg.matrixToXyzrpy(O2W)) :
                                    getFaceTransformTagOnly(LinAlg.matrixToXyzrpy(O2W)));

        // now compute transformation from observed robot to the observer
        double R2B[][] = LinAlg.multiplyMany(O2B, faceTransform);

        // observed robot -to- world transformation
        double R2W[][] = LinAlg.multiplyMany(Observer, R2B);

        return LinAlg.matrixToXyzrpy(R2W);
    }

    public static double[] getTagXyzrpy(TagDetection d,
                                        double Observer[][], double tagSize_m, boolean transformToRobotPose)
    {
        double O2B[][] = getTagToPose(d, tagSize_m);
        return getTagXyzrpy(O2B, Observer, transformToRobotPose);
    }

    // when following robots we want an xyt to the robot pose (not tag)
    public static double[][] getFaceTransformRobot(double xyzrpy[])
    {
        double faceTransform[][];

        if (Math.abs(xyzrpy[4] - Math.PI/2.0) < Math.PI/4.0) {
            // "clockwise 3" from printed tag orientation -- back
            faceTransform = LinAlg.multiplyMany(LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*0.0/2.0}),
                                                LinAlg.rollPitchYawToMatrix(new double[] {0, Math.PI/2.0, 0}),
                                                LinAlg.rollPitchYawToMatrix(new double[] {Math.PI, 0, 0}),
                                                LinAlg.translate(-0.165, 0.0000, -0.457)
                );
        } else if (Math.abs(xyzrpy[4] + Math.PI/2.0) < Math.PI/4.0) {
            // "clockwise 1" from printed tag orientation -- front
            faceTransform = LinAlg.multiplyMany(LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*2.0/2.0}),
                                                LinAlg.rollPitchYawToMatrix(new double[] {Math.PI, -Math.PI/2.0, 0}),
                                                LinAlg.rollPitchYawToMatrix(new double[] {Math.PI, 0, 0}),
                                                LinAlg.translate(-0.317, 0.0000, -0.457)
                                                );
        } else if (xyzrpy[3] > 0) {
            // "clockwise 0" from printed tag orientation -- right
            faceTransform = LinAlg.multiplyMany(LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*1.0/2.0}),
                                                LinAlg.rollPitchYawToMatrix(new double[] {-Math.PI/2.0, 0, Math.PI/2.0}),
                                                LinAlg.translate(-0.241,  0.076, -0.457)
                                                );
        } else {
            // "clockwise 2" from printed tag orientation -- left
            faceTransform = LinAlg.multiplyMany(LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*3.0/2.0}),
                                                LinAlg.rollPitchYawToMatrix(new double[] {Math.PI/2.0, 0, -Math.PI/2.0}),
                                                LinAlg.translate(-0.241, -0.076, -0.457)
                                                );
        }
        return faceTransform;
    }

    // for flipbook tags and camera following we want to point towards tag (not robot)
    public static double[][] getFaceTransformTagOnly(double xyzrpy[])
    {
        double faceTransform[][];

        if (Math.abs(xyzrpy[4] - Math.PI/2.0) < Math.PI/4.0) {
            // "clockwise 3" from printed tag orientation -- back
            faceTransform = LinAlg.multiplyMany(
                LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*0.0/2.0}),
                LinAlg.rollPitchYawToMatrix(new double[] {0, Math.PI/2.0, 0})
                // no rotation needed for back tag
                );
        } else if (Math.abs(xyzrpy[4] + Math.PI/2.0) < Math.PI/4.0) {
            // "clockwise 1" from printed tag orientation -- front
            faceTransform = LinAlg.multiplyMany(
                LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*2.0/2.0}),
                LinAlg.rollPitchYawToMatrix(new double[] {Math.PI, -Math.PI/2.0, 0}),
                LinAlg.rollPitchYawToMatrix(new double[] {0, 0, Math.PI})
                );
        } else if (xyzrpy[3] > 0) {
            // "clockwise 0" from printed tag orientation -- right
            faceTransform = LinAlg.multiplyMany(
                LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*1.0/2.0}),
                LinAlg.rollPitchYawToMatrix(new double[] {-Math.PI/2.0, 0, Math.PI/2.0}),
                LinAlg.rollPitchYawToMatrix(new double[] {0, 0, Math.PI/2})
                );
        } else {
            // "clockwise 2" from printed tag orientation -- left
            faceTransform = LinAlg.multiplyMany(
                LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI*3.0/2.0}),
                LinAlg.rollPitchYawToMatrix(new double[] {Math.PI/2.0, 0, -Math.PI/2.0}),
                LinAlg.rollPitchYawToMatrix(new double[] {0, 0, -Math.PI/2})
                );
        }
        return faceTransform;
    }

    public static double[] getTagPixels(double homography[][], double hxy[])
    {
        return new double[]{Math.round(homography[0][2] / homography[2][2] + hxy[0]),
                            Math.round(homography[1][2] / homography[2][2] + hxy[1])};
    }

    public static double getNumPixelsFromCenter(double homography[][], double hxy[])
    {
        double pxl[] = getTagPixels(homography, hxy);
        return  Math.sqrt(LinAlg.sq(pxl[0] - hxy[0]) + LinAlg.sq(pxl[1] - hxy[1]));
    }
}
