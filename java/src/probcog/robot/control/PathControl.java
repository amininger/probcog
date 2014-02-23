package probcog.robot.control;

import april.jmat.*;
import april.jmat.geom.*;

import probcog.lcmtypes.*;
import probcog.robot.robot.RobotDriver;

public class PathControl
{

    // Get a diff drive command to drive the CENTER of the robot along path
    public static diff_drive_t getDiffDrive(double [] pos_center,
                                            double [] orientation,
                                            GLineSegment2D path,
                                            Params params,
                                            double requestedSpeedLimit)
    {
        diff_drive_t diff_drive = new diff_drive_t();

        //by default, use both
        diff_drive.left_enabled = diff_drive.right_enabled = true;
        diff_drive.left = 0;
        diff_drive.right = 0;

        // Stopping conditions:
        //   1) If no poses have come
        //   2) If the path is malformed
        if (pos_center == null || orientation == null || path == null ||
            params.slowdownDist_m < 0.01 ||
            params.minLookahead_m < 0.0001) {
            return diff_drive;
        }

        // Outline
        // 1) Find closest segment to pose, closes point 'nearest'
        // 2) Find 'lookahead' point on path which intersects with a
        //    circle of radius lookahead dist centered at closestPoint
        // 3) Find min speed constraint over speedPreview_m after closestPoint
        // 4) Drive to 'lookahead'


        // 1)
        double pos[] = {pos_center[0], pos_center[1]};
        double [] closestPoint = path.closestPoint(pos);

        double lookahead = params.minLookahead_m;
        if (true) { //scoping
            // 1b)  Find the appropriate lookahead distance
            // double prev[] = path.xyr[closestIndex];
            // double next[] = path.xyr[closestIndex + 1];
            // prev[2] = Math.min(prev[2], 100*params.maxLookahead_m); //Enable doing math on radii
            // next[2] = Math.min(next[2], 100*params.maxLookahead_m);
            // double pDist = LinAlg.distance(LinAlg.resize(prev,2), closestPoint);
            // double nDist = LinAlg.distance(LinAlg.resize(next,2), closestPoint);

            // double linear = pDist / (pDist + nDist); //Interpolate linearly

            // lookahead = prev[2]*linear + next[2]*(1.0-linear);
            lookahead = MathUtil.clamp(lookahead,params.minLookahead_m, params.maxLookahead_m);
        }

        // 2) Find lookahead point
        double lookaheadPoint[] = LinAlg.resize(path.p2,2); //Default to end of path

        // 2b) find distance to the end of the path, and the combined speed limit
        double end[] = LinAlg.resize(path.p2, 2);
        double endDist = LinAlg.distance(pos, end);
        double endProp = MathUtil.clamp((endDist / params.slowdownDist_m), 0.0, 1.0);
        double endSpeedLimit = endProp*params.maxSpeed_p + (1.0 - endProp)*params.minMotorSpeed_p;

        if (endDist < params.destTolerance_m) //return here?
            endSpeedLimit = 0.0;

        // 4)
        double combinedSpeedLimit = Math.min(endSpeedLimit, requestedSpeedLimit);

        double cur_pos [] = {pos[0], pos[1],
                             LinAlg.quatToRollPitchYaw(orientation)[2]};

        double [] drive = getKProp(cur_pos, lookaheadPoint, params, combinedSpeedLimit);

        diff_drive.left  = drive[RobotDriver.LEFT];
        diff_drive.right = drive[RobotDriver.RIGHT];
        return diff_drive;
    }

    /**
     * Local path follower using proportional constants for angle
     * errors and distance.
     */
    private static double[] getKProp(double pos[], double target[],
                                     Params params,
                                     double maxSpeed)
    {
        assert(maxSpeed <= 1 && maxSpeed >= 0);

        double dx = target[0] - pos[0];
        double dy = target[1] - pos[1];
        double heading = pos[2];

        double angleToTarget = Math.atan2(dy, dx);

        // Compute how far we have to go:
        // 1. Distance to travel
        double rho   = MathUtil.clamp(Math.sqrt(dx*dx + dy*dy),
                                      0.0, params.slowdownDist_m);
        // 2. Heading difference to destination
        double alpha = MathUtil.mod2pi(angleToTarget - heading);

        // 3. Heading different to target heading
        double beta = MathUtil.mod2pi((-alpha) - heading);

        // Compute the forward and turning components of the trajectory separately
        double turn = params.turnRatio_p * alpha;
        double speed = 1.0;

        // When not facing the right direction, turn in place
        if (Math.abs(alpha) > params.turnInPlaceThresh_rad) {
            speed = 0;
            maxSpeed = Math.min(maxSpeed, .4);
        }

        double targetSpeeds[] = {speed - turn*RobotDriver.BASELINE_METERS,  // LEFT
                                 speed + turn*RobotDriver.BASELINE_METERS}; // RIGHT

        // Compute a norm [1,infty) to make sure we aren't exceeding a speed limit
        double norm = MathUtil.clamp(LinAlg.max(LinAlg.abs(targetSpeeds)),
                                     1.0, Double.MAX_VALUE);

        // Compute a speed scale based on distance to goal, max speed
        // and use norm to ensure that it is feasible
        double scale = maxSpeed / norm;

        double speeds[] = LinAlg.scale(targetSpeeds, scale);

        // ensures we don't overheat the motors
        if (Math.abs(speeds[0]) < params.minMotorSpeed_p && Math.abs(speeds[1]) < params.minMotorSpeed_p)
            speeds[0] = speeds[1] = 0.0;

        return speeds;
    }


}