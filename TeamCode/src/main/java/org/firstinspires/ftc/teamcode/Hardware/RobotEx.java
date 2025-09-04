package org.firstinspires.ftc.teamcode.Hardware;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;

public class RobotEx {
    public enum OpModeType {
        TELEOP, AUTO
    }

    public enum Alliance {
        RED,
        BLUE
    }

    protected OpModeType opModeType;
    protected Alliance alliance;

    protected FtcDashboard dashboard;

    protected GamepadExEx driverOp;
    protected GamepadExEx toolOp;

    protected MecanumDrive drive = null;
    protected IMUSubsystem gyro;

    protected Telemetry telemetry;
    public RobotEx(RobotMap robotMap, DriveConstants RobotConstants,
                   OpModeType type, Alliance alliance, Pose pose
    ) {
        this.alliance = alliance;

        initCommon(robotMap, RobotConstants, type, pose);

        if (type == OpModeType.TELEOP) {
            initTele(robotMap);
            opModeType = OpModeType.TELEOP;
        } else {
            initAuto(robotMap);
            opModeType = OpModeType.AUTO;
        }
    }

    public void initCommon(RobotMap robotMap, DriveConstants RobotConstants,
                           OpModeType type, Pose startingPose) {
        // --------------------------------------- Camera --------------------------------------- //
        this.dashboard = FtcDashboard.getInstance();

        // ------------------------------------- Telemetries ------------------------------------ //
        this.telemetry = robotMap.getTelemetry();

        // ---------------------------------------- Drive --------------------------------------- //
        drive = new MecanumDrive(robotMap, RobotConstants);

        // ----------------------------------------- IMU ---------------------------------------- //
        gyro = new IMUSubsystem(
                robotMap,
                () -> robotMap.getOdometry().getHeading(),
                Math.toDegrees(startingPose.getTheta())
        );

        CommandScheduler.getInstance().registerSubsystem(gyro);
    }

    public void initAuto(RobotMap robotMapUtil) {
        // ------------------------------------- Drivetrain ------------------------------------- //
        // TODO: Only if we run Auto in TeleOP

        // ----------------------- Setup and Initialize Mechanisms Objects ---------------------- //
        initMechanismsAutonomous();
    }

    public void initTele(RobotMap robotMap) {
        // -------------------------------------- Gamepads -------------------------------------- //
        this.driverOp = robotMap.getDriverOp();
        this.toolOp = robotMap.getToolOp();

        // ------------------------------------ Gyro Follower ----------------------------------- //
        driverOp.getGamepadButton(GamepadKeys.Button.START)
                .whenPressed(new InstantCommand(gyro::resetYawValue, gyro));

//        gyroTargetSubsystem = new HeadingControllerTargetSubsystem(driverOp::getRightX, driverOp::getRightY, telemetry);

//        gyroFollow = new HeadingControllerSubsystem(gyro::getYaw,
//                gyro::findClosestOrientationTarget, telemetry);

//        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER)
//                .whenPressed(new InstantCommand(gyroFollow::toggleState, gyroFollow));

//        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_STICK_BUTTON)
//                .whenPressed(new InstantCommand(drive::setFieldCentric, drive));
//
//        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON)
//                .whenPressed(new InstantCommand(drive::setRobotCentric, drive)); // TODO

        // ----------------------- Setup and Initialize Mechanisms Objects ---------------------- //
//        initMechanismsTeleOp();
    }

    // ------------------------------------- Drive Commands ------------------------------------- //
    public double drivetrainStrafe() {
        return driverOp.getLeftX();
    }

    public double drivetrainForward() {
        return driverOp.getLeftY();
    }

    public double drivetrainTurn() {
//        if (gyroFollow.isEnabled()) return -gyroFollow.calculateTurn();

        return driverOp.getRightX();
    }

    public void drive_update() {
        drive.drive(
                drivetrainForward(),
                drivetrainStrafe(),
                drivetrainTurn(),
                getHeading(),
                driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        );
    }

    // ------------------------------------- Drive Type Pick ------------------------------------ //
    public void setFieldCentric() {
        drive.setFieldCentric();
    }

    public void setRobotCentric() {
        drive.setRobotCentric();
    }

    public void drive_setEnabled(boolean enabled) {
        drive.setEnabled(enabled);
    }

    // -------------------------------- Mechanisms Initialization ------------------------------- //
    public void initMechanismsAutonomous() {
        // should be overridden by child class
    }

    public void initMechanismsTeleOp() {
        // should be overridden by child class
    }

    // ----------------------------------------- Getters ---------------------------------------- //
    public double getHeading() {
        return gyro.getRawYaw();
    }

    public double getContinuousHeading() {
        return gyro.getYaw();
    }

    public double getHeadingVelocity() {
        return 0.0; // TODO: Implement
    }

    public Alliance getAlliance() {
        return alliance;
    }
}