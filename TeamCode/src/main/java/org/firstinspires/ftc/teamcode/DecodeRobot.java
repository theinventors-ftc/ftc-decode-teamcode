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

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Drive.MecanumDrive;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.PinpointYawWrapper;
import org.firstinspires.ftc.teamcode.Mechanisms.Detection;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
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
    protected PinpointYawWrapper yawWrapper;

    private boolean hasInit = false;

    protected PinpointLocalizer teleOpLocalizer;

    // Mechanisms
    protected Intake intake;
    protected Passthough passthough;
    public static int fingerBetween = 80, fingerHold = 340;
    protected Shooter shooter;
    protected Detection detection;

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

    public MotifStorage.Motif getMotif() {
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

        intake = new Intake(robotMap, this::getHeading, this::drivetrainForward, this::drivetrainStrafe);
        passthough = new Passthough(robotMap, getMotif());
        shooter = new Shooter(
            robotMap,
            this::getPose,
            alliance
        );
        detection = new Detection(robotMap);

        toolOp.getGamepadButton(GamepadKeys.Button.RIGHT_BUMPER).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new InstantCommand(intake::intake, intake),
                        new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.INTAKE), passthough),
                        new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.INTAKE), passthough),
                        new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.INTAKE), passthough)
                ),
                new SequentialCommandGroup(
                        new InstantCommand(intake::stop, intake),
                        new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.HOLD), passthough),
                        new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.HOLD), passthough),
                        new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.HOLD), passthough)
                ),
                () -> intake.getState() != Intake.IntakeState.INTAKE
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.A).whenPressed((new ConditionalCommand(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.wheelsAtSpeed()),
                        new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.FEED), passthough),
                        new WaitCommand(fingerHold),
                        new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.HOLD), passthough)
                ),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange()
        )));

        toolOp.getGamepadButton(GamepadKeys.Button.B).whenPressed((new ConditionalCommand(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.wheelsAtSpeed()),
                        new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.FEED), passthough),
                        new WaitCommand(fingerHold),
                        new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.HOLD), passthough)
                ),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange()
        )));

        toolOp.getGamepadButton(GamepadKeys.Button.Y).whenPressed((new ConditionalCommand(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.wheelsAtSpeed()),
                        new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.FEED), passthough),
                        new WaitCommand(fingerHold),
                        new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.HOLD), passthough)
                ),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange()
        )));

        toolOp.getGamepadButton(GamepadKeys.Button.LEFT_BUMPER).whenPressed(new ConditionalCommand(
                new SequentialCommandGroup(
                        new WaitUntilCommand(() -> shooter.wheelsAtSpeed()),
                        new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.FEED), passthough),
                        new WaitCommand(fingerHold),
                        new InstantCommand(() -> passthough.setState(0, Passthough.FingerState.HOLD), passthough),
                        new WaitCommand(fingerBetween),
                        new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.FEED), passthough),
                        new WaitCommand(fingerHold),
                        new InstantCommand(() -> passthough.setState(1, Passthough.FingerState.HOLD), passthough),
                        new WaitCommand(fingerBetween),
                        new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.FEED), passthough),
                        new WaitCommand(fingerHold),
                        new InstantCommand(() -> passthough.setState(2, Passthough.FingerState.HOLD), passthough),
                        new WaitCommand(fingerBetween)
                ),
                new InstantCommand(),
                () -> shooter.turretInRange() && shooter.inLUTRange()
        ));

        toolOp.getGamepadButton(GamepadKeys.Button.DPAD_UP).whenPressed(new ConditionalCommand(
                new InstantCommand(shooter::enableWheels, shooter),
                new InstantCommand(shooter::disableWheels, shooter),
                () -> !shooter.areWheelsEnabled()
        ));
    }
}
