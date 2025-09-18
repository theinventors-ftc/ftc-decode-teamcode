package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Commands.ShootingCommand;
import org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems.Fingers;
import org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems.Intake;
import org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems.Shooter;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.IMUSubsystem;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;

public class DecodeRobot {
    public enum Alliance {
        RED,
        BLUE
    }
    protected Alliance alliance;

    protected FtcDashboard dashboard;
    protected GamepadExEx driverOp;
    protected GamepadExEx toolOp;
    protected Telemetry telemetry;

    protected MecanumDrive drive = null;
    protected IMUSubsystem gyro;

    private boolean hasInit = false;

    // Mechanisms
    protected Intake intake;
    protected Shooter shooter;
    protected Fingers fingers;
    protected RobotMap robotMap;

    public DecodeRobot(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance, Pose pose
    ) {
        this.alliance = alliance;
        this.robotMap = robotMap;

        initCommon(robotMap, driveConstants);
        initTele(robotMap, pose);

        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
            Math.abs(drivetrainStrafe()) > 0.1 ||
            Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
            .whenActive(new InstantCommand(this::initMechanismsTeleOp));
    }

    public DecodeRobot(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance
    ) {
        this.alliance = alliance;

        initCommon(robotMap, driveConstants);
        initAuto();
    }

    /*-- Drive Commands --*/
    public double drivetrainStrafe() {
        return driverOp.getLeftX();
    }

    public double drivetrainForward() {
        return driverOp.getLeftY();
    }

    public double drivetrainTurn() {
        return driverOp.getRightX();
    }

    public void drive_update() {
        drive.drive(
            drivetrainStrafe(),
            drivetrainForward(),
            drivetrainTurn(),
            getHeading(),
            driverOp.getTrigger(GamepadKeys.Trigger.LEFT_TRIGGER)
        );
    }

    public void drive_update(Pose pose) {
        drive.drive(
            pose.getX(),
            pose.getY(),
            pose.getTheta(),
            0,
            0
        );
    }

    /*-- Drive Type Pick --*/
    public void setFieldCentric() {
        drive.setFieldCentric();
    }

    public void setRobotCentric() {
        drive.setRobotCentric();
    }

    public void setAutoEnabled(boolean enabled) {
        drive.setAutoEnabled(enabled);
    }

    /*-- Getters --*/
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

    /*-- Initializations --*/
    public void initCommon(RobotMap robotMap, DriveConstants driveConstants) {
        //- Camera
        this.dashboard = FtcDashboard.getInstance();

        //- Telemetries
        this.telemetry = robotMap.getTelemetry();

        //- Drive
        drive = new MecanumDrive(robotMap, driveConstants);
    }

    public void initAuto() {
        //- Setup and Initialize Mechanisms Objects
        initMechanismsAutonomous();
    }

    public void initTele(RobotMap robotMap, Pose startingPose) {
        //- IMU
        //TODO: turn this into the pinpoint heading
        gyro = new IMUSubsystem(
            robotMap,
            () -> 0,
            0
        );

//        gyro = new IMUSubsystem(
//                robotMap,
//                () -> robotMap.getOdometry().getHeading(),
//                Math.toDegrees(startingPose.getTheta())
//        );

        CommandScheduler.getInstance().registerSubsystem(gyro);

        //- Gamepads
        this.driverOp = robotMap.getDriverOp();
        this.toolOp = robotMap.getToolOp();
    }

    /*-- Mechanisms Initialization --*/
    public void initMechanismsAutonomous() {
        //TODO: make init Mechanisms
    }

    public void initMechanismsTeleOp() {
        hasInit = true;

        intake = new Intake(robotMap.getHm());
        shooter = new Shooter(robotMap.getHm());
        fingers = new Fingers(robotMap.getHm(), telemetry);

        // Procedures
        driverOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(
                new ConditionalCommand(
                        new SequentialCommandGroup(
                                new InstantCommand(shooter::intake),
                                new InstantCommand(() -> fingers.move(Fingers.State.INTAKE)),
                                new InstantCommand(intake::intake)
                        ),
                        new SequentialCommandGroup(
                                new InstantCommand(shooter::stop),
                                new InstantCommand(() -> fingers.move(Fingers.State.PARK)),
                                new WaitCommand(300),
                                new InstantCommand(intake::reverse),
                                new WaitCommand(500),
                                new InstantCommand(intake::stop)
                        ),
                        () -> intake.getState() != Intake.State.INTAKE
                )
        );

        driverOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(
                new ShootingCommand(shooter, fingers)
        );
    }
}
