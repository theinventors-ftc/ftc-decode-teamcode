package org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated;

import com.acmerobotics.dashboard.config.Config;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCon;

@Config
public class RobotConstants {

    private static double

        /*-- Localization --*/
        forwardY = -1.2,
        strafeX = -5.75,

        /*-- Robot Movement --*/
        maxVelocity = 10, // max target velocity for the path follow (ticks per second)
        maxAcceleration = 7, // max target acceleration and deceleration for the path follow
    // (ticks per second)
        maxDecceleration = 7, // max target acceleration and deceleration for the path follow
    // (ticks per second)
                                // POSITIVE ONLY

        maxRotationalVelocity = 180, // max rotational target velocity for the path follow
        maxRotationalAcceleration = 1, // max rotational target acceleration and deceleration for the path follow
        maxRotationalDecceleration = 1, // max rotational target acceleration and deceleration for the path follow
                                // POSITIVE ONLY

        minRadiusRange = 1, // min lookahead distance (inches)
        maxRadiusRange = 5, // max lookahead distance (inches)

        xThreshold = 3, // threshold for X axis in the path following algorithm
        yThreshold = 3, // threshold for Y axis in the path following algorithm
        thetaThreshold = 3, // threshold for Theta(rotational) axis in the path following algorithm

        hybridThetaDistanceThreshold = 7, // absolut distance threshold of the hybrid theta interpolation

        lowerPIDThreshold_X = 4, // inches from ending target to activate the lower PIDS
        lowerPIDThreshold_Y = 4, // inches from ending target to activate the lower PIDS
        rotationalLowerPIDThreshold = 0, // degrees from ending target to activate the lower PID

        robotX = 15, // robot's size in the x axis
        robotY = 15; // robot's size in the y axis

    /*-- Follower --*/
    public static PIDFExCon

        upperParallelPID = new PIDFExCon(
            0.1,
            0.0,
            0.0,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        ),
        lowerParallelPID = new PIDFExCon(
            0.05,
            0.0,
            0.0,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        ),
        upperPerpendicularPID = new PIDFExCon(
            0.1,
            0.0,
            0.0,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        ),
        lowerPerpendicularPID = new PIDFExCon(
            0.05,
            0.0,
            0.0,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        ),
        upperRotationalPID = new PIDFExCon(
            0.04,
            0.0,
            0.0,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        ),
        lowerRotationalPID = new PIDFExCon(
            0.01,
            0.0,
            0.0,
            0,
            0.1,
            0.0,
            0.0,
            0.0
        );

    /*-- Meow --*/
    public static void setMaxVelocity(double set) {
        RobotConstants.maxVelocity = set;
    }
    public static void setMaxAcceleration(double set) {
        RobotConstants.maxAcceleration = set;
    }
    public static void setMaxDecceleration(double set) {
        RobotConstants.maxDecceleration = set;
    }
    public static void setMaxRotationalVelocity(double set) {
        RobotConstants.maxRotationalVelocity = set;
    }
    public static void setMaxRotationalAcceleration(double set) {
        RobotConstants.maxRotationalAcceleration = set;
    }
    public static void setMaxRotationalDecceleration(double set) {
        RobotConstants.maxRotationalDecceleration = set;
    }
    public static void setMinRadiusRange(double set) {
        RobotConstants.minRadiusRange = set;
    }
    public static void setMaxRadiusRange(double set) {
        RobotConstants.maxRadiusRange = set;
    }
    public static void setRobotX(double set) {
        RobotConstants.robotX = set;
    }
    public static void setRobotY(double set) {
        RobotConstants.robotY = set;
    }
    public static void setForwardY(double set) {
        RobotConstants.forwardY = set;
    }
    public static void setStrafeX(double set) {
        RobotConstants.strafeX = set;
    }
    public static void setX_Threshold (double set) {
        RobotConstants.xThreshold = set;
    }
    public static void setY_Threshold (double set) {
        RobotConstants.yThreshold = set;
    }
    public static void setTheta_Threshold (double set) {
        RobotConstants.thetaThreshold = set;
    }
    public static void setHybridThetaDistanceThreshold(double set) {
        RobotConstants.hybridThetaDistanceThreshold = set;
    }
    public static void setUpperParallelPID(PIDFExCon set) {
        RobotConstants.upperParallelPID = set;
    }
    public static void setLowerParallelPID(PIDFExCon set) {
        RobotConstants.lowerParallelPID = set;
    }
    public static void setUpperPerpendicularPID(PIDFExCon set) {
        RobotConstants.upperPerpendicularPID = set;
    }
    public static void setLowerPerpendicularPID(PIDFExCon set) {
        RobotConstants.lowerPerpendicularPID = set;
    }
    public static void setUpperRotationalPID(PIDFExCon set) {
        RobotConstants.upperRotationalPID = set;
    }
    public static void setLowerRotationalPID(PIDFExCon set) {
        RobotConstants.lowerRotationalPID = set;
    }
    public static void setLowerPIDThreshold_X(double set) {
        RobotConstants.lowerPIDThreshold_X = set;
    }
    public static void setLowerPIDThreshold_Y(double set) {
        RobotConstants.lowerPIDThreshold_Y = set;
    }
    public static void setRotationalLowerPIDThreshold(double set) {
        RobotConstants.rotationalLowerPIDThreshold = set;
    }

    /*-- Meow meow --*/
    public static double getMaxVelocity() {
        return maxVelocity;
    }
    public static double getMaxAcceleration() {
        return maxAcceleration;
    }
    public static double getMaxDecceleration() {
        return maxDecceleration;
    }
    public static double getMaxRotationalVelocity() {
        return maxRotationalVelocity;
    }
    public static double getMaxRotationalAcceleration() {
        return maxRotationalAcceleration;
    }
    public static double getMaxRotationalDecceleration() {
        return maxRotationalDecceleration;
    }
    public static double getMinRadiusRange() {
        return minRadiusRange;
    }
    public static double getMaxRadiusRange() {
        return maxRadiusRange;
    }
    public static double getRobotX() {
        return robotX;
    }
    public static double getRobotY() {
        return robotY;
    }
    public static double getForwardY() {
        return forwardY;
    }
    public static double getStrafeX() {
        return strafeX;
    }
    public static double getX_Threshold () {
        return xThreshold;
    }
    public static double getY_Threshold () {
        return yThreshold;
    }
    public static double getTheta_Threshold () {
        return thetaThreshold;
    }
    public static double getHybridThetaDistanceThreshold() {
        return hybridThetaDistanceThreshold;
    }
    public static PIDFExCon getUpperParallelPID() {
        return upperParallelPID;
    }
    public static PIDFExCon getLowerParallelPID() {
        return lowerParallelPID;
    }
    public static PIDFExCon getUpperPerpendicularPID() {
        return upperPerpendicularPID;
    }
    public static PIDFExCon getLowerPerpendicularPID() {
        return lowerPerpendicularPID;
    }
    public static PIDFExCon getUpperRotationalPID() {
        return upperRotationalPID;
    }
    public static PIDFExCon getLowerRotationalPID() {
        return lowerRotationalPID;
    }
    public static double getLowerPIDThreshold_X() {
        return lowerPIDThreshold_X;
    }
    public static double getLowerPIDThreshold_Y() {
        return lowerPIDThreshold_Y;
    }
    public static double getRotationalLowerPIDThreshold() {
        return rotationalLowerPIDThreshold;
    }
}