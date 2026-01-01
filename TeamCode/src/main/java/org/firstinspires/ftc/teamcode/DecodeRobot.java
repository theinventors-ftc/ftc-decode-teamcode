package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.IMUSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.PinpointYawWrapper;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.PinpointLocalizer;

public class DecodeRobot {
    public enum Alliance {
        RED,
        BLUE
    }
    protected Alliance alliance;

    protected FtcDashboard dashboard;
    protected GamepadExEx driverOp, toolOp;
    protected Telemetry telemetry;

    protected MecanumDrive drive = null;
    protected PinpointYawWrapper yawWrapper;

    private boolean hasInit = false;

    protected PinpointLocalizer teleOpLocalizer;

    // Mechanisms
    protected Intake intake;
    protected Passthough passthough;
    protected Shooter shooter;

    protected MotifStorage.MotifState motif;

    public DecodeRobot(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance,
                       Pose pose, MotifStorage.MotifState motif
    ) {
        this.alliance = alliance;
        this.motif = motif;

        initCommon(robotMap, driveConstants);
        initTele(robotMap, pose);

        // Init Mechanisms when driver starts moving the robot
        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
            Math.abs(drivetrainStrafe()) > 0.1 ||
            Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
            .whenActive(new InstantCommand(() -> this.initMechanismsTeleOp(robotMap)));
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
        teleOpLocalizer.update();

        telemetry.addData("Pose", "X: %.2f, Y: %.2f, Theta: %.2f",
            getPose().getX(), getPose().getY(), getPose().getTheta());

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
        return yawWrapper.getRawYaw();
    }

    public double getContinuousHeading() {
        return yawWrapper.getContinuousYaw();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public MotifStorage.MotifState getMotif() {
        return motif;
    }

    public Pose getPose() {
        return teleOpLocalizer.getPose();
    }
    public Pose getPoseVelocity() {
        return teleOpLocalizer.getVelocity();
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
        teleOpLocalizer = new PinpointLocalizer(robotMap, startingPose);

        yawWrapper = new PinpointYawWrapper(
            robotMap,
            () -> MathFunction.wrapDegrees(getPose().getTheta())
        );
        CommandScheduler.getInstance().registerSubsystem(yawWrapper);

        //- Gamepads
        this.driverOp = robotMap.getDriverOp();
        this.toolOp = robotMap.getToolOp();
    }

    /*-- Mechanisms Initialization --*/
    public void initMechanismsAutonomous() {
        //TODO: make init Mechanisms
    }

    public void initMechanismsTeleOp(RobotMap robotMap) {
        hasInit = true;

        driverOp.getGamepadButton(GamepadKeys.Button.START).whenPressed(yawWrapper::resetYawValue);

        intake = new Intake(robotMap);
//        passthough = new Passthough(robotMap, getMotif());
        shooter = new Shooter(
            robotMap,
            this::getPose,
            alliance,
            telemetry
        );

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ConditionalCommand(
                new InstantCommand(intake::intake),
                new InstantCommand(intake::stop),
                () -> intake.getState() == Intake.IntakeState.STOPPED
        ));

        // koympia
    }
}
