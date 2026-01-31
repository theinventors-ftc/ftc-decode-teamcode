package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.geometry.CoordinateSystem;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.TelemetryManager;
import com.bylazar.telemetry.PanelsTelemetry;

import org.firstinspires.ftc.teamcode.Mechanisms.CommandSeriesVault;
import org.firstinspires.ftc.teamcode.Mechanisms.Intake;
import org.firstinspires.ftc.teamcode.Mechanisms.Passthough;
import org.firstinspires.ftc.teamcode.Mechanisms.Shooter;
import org.firstinspires.ftc.teamcode.pedroPathing.Constants;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.pedropathing.geometry.Pose;

@Autonomous(name = "AutonomousTest", group = "Autonomous")
@Configurable
public class TestAuto extends CommandOpMode {
    private TelemetryManager panelsTelemetry;
    public Follower follower;
    private RobotMap robotMap;

    private Intake intake;
    private Passthough passthough;
    private Shooter shooter;
    private CommandSeriesVault commandVault;
    private SequentialCommandGroup temp;
    private enum PathState {
        GOAL,
        INTAKE1,
        LAUNCH1,
        AIM2,
        INTAKE2,
        LAUNCH2,
        INTAKE3,
        LAUNCHSMALL,
        PARKING,
        patata
    }
    private PathState pathState;

    private Paths paths;

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset(); // Ultra SOS
        panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
        robotMap = new RobotMap(hardwareMap, telemetry,null,null);

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(107.6, 135, Math.toRadians(0)));

        paths = new Paths(follower);

        setPathState(PathState.GOAL);

        intake = new Intake(robotMap,() -> 0, () -> 0, () -> 0);
        passthough = new Passthough(robotMap, MotifStorage.Motif.PPG);
        shooter = new Shooter(robotMap, this::getPoseFTCCoor, DecodeRobot.Alliance.RED, false);
        commandVault = new CommandSeriesVault(intake, passthough, shooter);
        commandVault.enableWheels().schedule();

        panelsTelemetry.debug("Status", "Initialized");
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void run() {
        super.run();
        autonomousPathUpdate();
        follower.update();

        for (LynxModule hub : robotMap.getHubs()) hub.clearBulkCache();

        // Log values to Panels and Driver Station
//        panelsTelemetry.debug("Path State", pathState);
//        telemetry.addData("X", follower.getPose().getX());
//        telemetry.addData("Y", follower.getPose().getY());
//        telemetry.addData("Heading", follower.getPose().getHeading());
//        telemetry.update();

        telemetry.addData("X", getPoseFTCCoor().getX());
        telemetry.addData("Y", getPoseFTCCoor().getY());
        telemetry.addData("Heading", getPoseFTCCoor().getTheta());
        telemetry.addData("PathState", pathState);
        telemetry.update();
    }


    public static class Paths {
        public PathChain
                StartToGoal,
                GoalToIntakeStack1,
                Intake1ToLauchArea1,
                LaunchArea1ToAimStack2,
                AimStack2ToIntakeStack2,
                IntakeStack2ToLaunchArea2,
                LauchArea2ToIntakeStack3,
                IntakeStack3ToSmallLaunchArea,
                SmallLaunchAreaToParking;

        public Paths(Follower follower) {
            StartToGoal = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(107.6, 135.0),
                                    new Pose(103.7, 120.7)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            GoalToIntakeStack1 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(103.7, 120.7),
                                    new Pose(84.5, 81.0),
                                    new Pose(93.6, 82.5),
                                    new Pose(126.0, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(0))
                    .build();

            Intake1ToLauchArea1 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(128.0, 83.5),
                                    new Pose(87.9, 83.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(300))
                    .build();

            LaunchArea1ToAimStack2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(87.9, 83.5),
                                    new Pose(92.016, 64.986),
                                    new Pose(102.5, 59.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(300), Math.toRadians(0))
                    .build();

            AimStack2ToIntakeStack2 = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(102.5, 59.5),
                                    new Pose(133.0, 59.5)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            IntakeStack2ToLaunchArea2 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(135.0, 59.5),
                                    new Pose(107.0, 55.8),
                                    new Pose(81.8, 70.5)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(270))
                    .build();

            LauchArea2ToIntakeStack3 = follower.pathBuilder().addPath(
                            new BezierCurve(
                                    new Pose(81.8, 70.5),
                                    new Pose(83.4, 36.1),
                                    new Pose(98.7, 35.075),
                                    new Pose(132.7, 35.6)
                            )
                    ).setTangentHeadingInterpolation()
                    .build();

            IntakeStack3ToSmallLaunchArea = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(132.7, 35.6),
                                    new Pose(84.9, 12.8)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(0), Math.toRadians(25))
                    .build();

            SmallLaunchAreaToParking = follower.pathBuilder().addPath(
                            new BezierLine(
                                    new Pose(84.9, 12.8),
                                    new Pose(108.0, 13.0)
                            )
                    ).setLinearHeadingInterpolation(Math.toRadians(25), Math.toRadians(0))
                    .build();
        }
    }

    public void autonomousPathUpdate() {
        switch (pathState) {
            case GOAL:
                follower.followPath(paths.StartToGoal);
                setPathState(PathState.INTAKE1);
                break;

            case INTAKE1:
                if(!follower.atParametricEnd()) break;

                if (!CommandScheduler.getInstance().isScheduled(temp)) {
//                    temp = new SequentialCommandGroup(commandVault.feedAllFingers());
                    temp = new SequentialCommandGroup(new WaitCommand(2000));
                    temp.schedule();
                }

                if(CommandScheduler.getInstance().isScheduled(temp)) break;

                temp = commandVault.startIntakeProc();
                temp.schedule();

                follower.followPath(paths.GoalToIntakeStack1);
                setPathState(PathState.LAUNCH1);

//                if(!follower.isBusy() && temp.isFinished()) {
//
//                    temp = commandVault.startIntakeProc();
//                    temp.schedule();
//
//                    follower.followPath(paths.GoalToIntakeStack1);
//                    setPathState(PathState.LAUNCH1);
//                }
                break;

            case LAUNCH1:
                if(!follower.isBusy()) {
                    // Color Sensors
                    follower.followPath(paths.Intake1ToLauchArea1);
                    setPathState(PathState.AIM2);
                }
                break;

            case AIM2:
                if(!follower.isBusy()) {
                    temp = new SequentialCommandGroup(
                            commandVault.stopIntakeProc(),
                            commandVault.feedAllFingers()
                    );
                    temp.schedule();
                    if (!temp.isFinished()) break;

                    temp = commandVault.startIntakeProc();
                    temp.schedule();

                    follower.followPath(paths.LaunchArea1ToAimStack2);
                    setPathState(PathState.INTAKE2);
                }
                break;

            case INTAKE2:
                if(!follower.isBusy()) {
                    follower.followPath(paths.AimStack2ToIntakeStack2);
                    setPathState(PathState.LAUNCH2);
                }
                break;

            case LAUNCH2:
                if(!follower.isBusy()) {
                    follower.followPath(paths.IntakeStack2ToLaunchArea2);
                    setPathState(PathState.INTAKE3);
                }
                break;

            case INTAKE3:
                if(!follower.isBusy()) {
                    temp = new SequentialCommandGroup(
                            commandVault.stopIntakeProc(),
                            commandVault.feedAllFingers()
                    );
                    temp.schedule();
                    if (!temp.isFinished()) break;

                    temp = commandVault.startIntakeProc();
                    temp.schedule();

                    follower.followPath(paths.LauchArea2ToIntakeStack3);
                    setPathState(PathState.LAUNCHSMALL);
                }
                break;

            case LAUNCHSMALL:
                if(!follower.isBusy()) {
                    follower.followPath(paths.IntakeStack3ToSmallLaunchArea);
                    setPathState(PathState.PARKING);
                }
                break;

            case PARKING:
                if(!follower.isBusy()) {
                    temp = new SequentialCommandGroup(
                            commandVault.stopIntakeProc(),
                            commandVault.feedAllFingers()
                    );
                    temp.schedule();

                    if (!temp.isFinished()) break;

                    follower.followPath(paths.SmallLaunchAreaToParking);
                    setPathState(PathState.patata);
                }
                break;

            case patata:
//                if(!follower.isBusy()) {
//                    setPathState(PathState.patata);
//                }
                break;
        }
    }

    public void setPathState(PathState pState) {
        pathState = pState;
    }

    public org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose getPoseFTCCoor() {
        Pose pedroPose = new Pose(
                follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getPose().getHeading()
        ).getAsCoordinateSystem(FTCCoordinates.INSTANCE);

        return new org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose(
                pedroPose.getX(),
                pedroPose.getY(),
                Math.toDegrees(pedroPose.getHeading())
        );
    }
}