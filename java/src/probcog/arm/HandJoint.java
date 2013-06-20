package probcog.arm;

import java.awt.Color;
import javax.swing.*;

import april.jmat.*;
import april.sim.*;
import april.util.*;
import april.vis.*;

import probcog.lcmtypes.*;

public class HandJoint implements Joint
{
    // Public parameters
    static double HAND_SPEED = 0.15;

    // Current VisObject
    VisObject vobj;

    // Current shape
    Shape shape;

    // XXX DEBUG
    public BoxShape staticBox;
    public BoxShape mobileBox;

    // State
    double[][] rotation;
    double[][] translation;

    Parameters params;

    // Finger length and positioning
    // XXX Only good for one gripper this way
    static final double mfY      = 0.100; //0.082;
    static final double moZ      = 0.014;
    static final double moY      = 0.014;
    static final double sfZ      = 0.070;
    static final double soZ      = 0.030;
    static final double soY      = 0.050; //0.040;

    // Finger separation and dimensions
    static final double ssep     = 0.015;
    static final double msep     = 0.020;
    static final double width    = 0.012;
    static final double height   = 0.008;


    static public class Parameters
    {
        public double dAngle = 0.0;
        public double aAngle = 0.0;
        public double length = 0.111;
        public double rMin = Math.toRadians(-40.0);
        public double rMax = Math.toRadians(120.0);
    }

    public HandJoint(Parameters params_)
    {
        params = params_;
        updateVisObject();

        rotation = LinAlg.identity(4);
        translation = LinAlg.translate(0,0,params.length);
    }

    public VisObject getVis()
    {
        return vobj;
    }

    public double[][] getRotation()
    {
        return rotation;
    }

    public double[][] getTranslation()
    {
        return translation;
    }

    public void setPos(double val)
    {
        params.dAngle = MathUtil.clamp(val, params.rMin, params.rMax);
    }

    public void updatePos(double val)
    {
        params.aAngle = val;
        // XXX Rotation update, if it was relevant
        updateVisObject();
    }

    public dynamixel_command_t getArmCommand()
    {
        dynamixel_command_t cmd = new dynamixel_command_t();
        cmd.position_radians = MathUtil.mod2pi(MathUtil.clamp(params.dAngle, params.rMin, params.rMax));

        cmd.speed = HAND_SPEED;
        cmd.max_torque = 0.5;

        // XXX Receiver sets utimes
        return cmd;
    }

    public Shape getShape()
    {
        updateVisObject();
        return shape;
    }

    /** Return the general position of the static fingers */
    public double[][] getStaticFingerPose()
    {
        double[][] xform = LinAlg.translate(0, soY, sfZ/2+soZ);
        return xform;
    }

    /** Return the general position of the mobile fingers */
    public double[][] getMobileFingerPose()
    {
        double[][] xform = LinAlg.rotateX(-params.aAngle);
        LinAlg.timesEquals(xform, LinAlg.translate(0, -mfY/2-moY, -moZ));
        return xform;
    }

    // ===================
    private void updateVisObject()
    {
        double ch = 0.03;
        double cr = 0.01;

        staticBox = new BoxShape(ssep+2*height, width, sfZ);
        mobileBox = new BoxShape(2*msep+3*height, mfY, width);
        shape = mobileBox;

        VzBox finger = new VzBox(1,1,1, new VzMesh.Style(Color.blue));
        VzCylinder cyl = new VzCylinder(cr, ch, new VzMesh.Style(Color.red));

        double[][] staticPose = getStaticFingerPose();
        double[][] mobilePose = getMobileFingerPose();

        // Static fingers
        VisChain static0 = new VisChain(staticPose,
                                        LinAlg.translate(-ssep/2,0,0),
                                        //LinAlg.translate(0,0,sfZ/2),
                                        //LinAlg.translate(-ssep/2,soY,soZ),
                                        LinAlg.scale(height,width,sfZ),
                                        finger);
        VisChain static1 = new VisChain(staticPose,
                                        LinAlg.translate(ssep/2,0,0),
                                        //LinAlg.translate(0,0,sfZ/2),
                                        //LinAlg.translate(ssep/2,soY,soZ),
                                        LinAlg.scale(height,width,sfZ),
                                        finger);

        // Mobile Fingers
        VisChain mobile0 = new VisChain(mobilePose,
                                        LinAlg.translate(-msep,0,0),
                                        //LinAlg.rotateX(-params.aAngle),
                                        //LinAlg.translate(0,-mfY/2,0),
                                        //LinAlg.translate(-msep,-moY,-moZ),
                                        LinAlg.scale(height,mfY,width),
                                        finger);
        VisChain mobile1 = new VisChain(mobilePose,
                                        //LinAlg.rotateX(-params.aAngle),
                                        //LinAlg.translate(0,-mfY/2,0),
                                        //LinAlg.translate(0,-moY,-moZ),
                                        LinAlg.scale(height,mfY,width),
                                        finger);
        VisChain mobile2 = new VisChain(mobilePose,
                                        LinAlg.translate(msep,0,0),
                                        //LinAlg.rotateX(-params.aAngle),
                                        //LinAlg.translate(0,-mfY/2,0),
                                        //LinAlg.translate(msep,-moY,-moZ),
                                        LinAlg.scale(height,mfY,width),
                                        finger);


        // Joint
        VisChain joint = new VisChain(LinAlg.rotateX(-params.aAngle),
                                      LinAlg.rotateY(Math.PI/2),
                                      LinAlg.translate(0,-moY,0),
                                      cyl);




        VisChain hand = new VisChain(static0,
                                     static1,
                                     mobile0,
                                     mobile1,
                                     mobile2,
                                     joint);

        vobj = hand;
    }

    /** For planning purposes, define a "length" for the hand where we
     *  model it as a rigid segment.
     */
    public double getLength()
    {
        return params.length;
    }

    public double getActualValue()
    {
        return params.aAngle;
    }

    public double getDesiredValue()
    {
        return params.dAngle;
    }

    public double getMinValue()
    {
        return params.rMin;
    }

    public double getMaxValue()
    {
        return params.rMax;
    }
}
