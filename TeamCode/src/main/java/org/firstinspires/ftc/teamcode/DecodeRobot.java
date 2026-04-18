package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.IMUSubsystem;
import org.firstinspires.ftc.teamcode.Hardware.PinpointYawWrapper;
import org.firstinspires.ftc.teamcode.Mechanisms.CommandSeriesVault;
import org.firstinspires.ftc.teamcode.Mechanisms.Detection;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Vector;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.PinpointLocalizer;

@Config
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
    protected IMUSubsystem imuSubsystem;
    private boolean hasInit = false;

    // Mechanisms
    protected Intake intake;
    protected Passthough passthough;
    protected Shooter shooter;
    protected CommandSeriesVault commandSeriesVault;

    protected MotifStorage.Motif motif;

    public DecodeRobot(RobotMap robotMap, DriveConstants driveConstants, Alliance alliance,
                       Pose pose, MotifStorage.Motif motif
    ) {
        this.alliance = alliance;
        this.motif = motif;

        initCommon(robotMap, driveConstants);
        initTele(robotMap, pose);
        this.initMechanismsTeleOp(robotMap);

//        // Init Mechanisms when driver starts moving the robot
//        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
//            Math.abs(drivetrainStrafe()) > 0.1 ||
//            Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
//            .whenActive(new InstantCommand(() -> this.initMechanismsTeleOp(robotMap)));
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
        telemetry.addData("Theta: ", "%.2f", getContinuousHeading());
        telemetry.addData("Alliance: ", getAlliance());

        drive.drive(
            drivetrainStrafe(),
            drivetrainForward(),
            drivetrainTurn(),
            getHeading() - (alliance == Alliance.RED ? -90 : 90),
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
        return imuSubsystem.getRawYaw();
    }

    public double getContinuousHeading() {
        return imuSubsystem.getYaw();
    }

    public Alliance getAlliance() {
        return alliance;
    }

    public MotifStorage.Motif getMotif() {
        return motif;
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
        //- Gamepads
        this.driverOp = robotMap.getDriverOp();
        this.toolOp = robotMap.getToolOp();

        imuSubsystem = new IMUSubsystem(robotMap, 0);
    }

    /*-- Mechanisms Initialization --*/
    public void initMechanismsAutonomous() {
        //TODO: make init Mechanisms
    }

    public void initMechanismsTeleOp(RobotMap robotMap) {
        hasInit = true;

        intake = new Intake(robotMap);
        passthough = new Passthough(robotMap, getMotif());
        shooter = new Shooter(
                robotMap,
                null,
                alliance,
                true,
                this::getContinuousHeading
        );

        commandSeriesVault = new CommandSeriesVault(intake, passthough, shooter);

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ConditionalCommand(
                commandSeriesVault.startIntakeProc(),
                commandSeriesVault.stopIntakeProc(),
                () -> intake.getState() != Intake.IntakeState.INTAKE
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed(new ConditionalCommand(
                commandSeriesVault.feedAllFingers(new Pose(48, 0, 0)),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed(new ConditionalCommand(
                commandSeriesVault.feedAllFingers(new Pose(0, 0, 0)),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed(new ConditionalCommand(
                commandSeriesVault.feedAllFingers(new Pose(-48, 0, 0)),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange() && shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.X).whenPressed(new ConditionalCommand(
                commandSeriesVault.enableWheels(),
                commandSeriesVault.disableWheels(),
                () -> !shooter.areWheelsEnabled()
        ));

        toolOp.getGamepadButton((GamepadKeys.Button.RIGHT_STICK_BUTTON)).whenPressed(
                commandSeriesVault.reverseIntake()
        );

        toolOp.getGamepadButton((GamepadKeys.Button.RIGHT_STICK_BUTTON)).whenReleased(
                commandSeriesVault.stopIntake()
        );

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_STICK_BUTTON).whenPressed(
                commandSeriesVault.rearrangeArtifacts()
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(
                commandSeriesVault.flickFrontFinger()
        );

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_DOWN).whenPressed(
                commandSeriesVault.flickRearFinger()
        );
    }
}
