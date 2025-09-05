package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.Drive.DecodeRobot;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.RobotConstants;
import org.firstinspires.ftc.teamcode.PurePursuit.RobotMovement;
import org.firstinspires.ftc.teamcode.Util.Timer;

@Disabled
@Autonomous(name = "Do not run this AutoOp", group = "")
public class AutoOpBase extends CommandOpMode {
    private DriveConstants driveConstants;
    private DecodeRobot robot;
    private RobotMovement robotMovement;
    private RobotMap robotMap;
    private Timer timer;

    /* -- States -- */
    private enum PathState {
        FIRST,
        SECOND,
        STOP,
        patata
    }
    private PathState pathState;

    /* -- Poses and Vectors -- */
    private Pose startingPose = new Pose(0, 0, 180);

    /* -- Paths -- */
    private Pose[] currentPath;

    private Pose[] first = {
        startingPose,
        new Pose(50, 0,0),
        new Pose(50, -80,0),
        new Pose(100, -80,0),
    };

    private Pose[] second = {
        new Pose(0,0,0),
        new Pose(20, 0,0),
        new Pose(0, 40,270),
        new Pose(0,0,270)
    };

    /* -- Extra Util Functions -- */
    public void setPathState(PathState pState) {
        pathState = pState;
        timer.resetTimer();
    }

    public void autoPathUpdate() {
        switch (pathState) {
            case FIRST:
                robotMovement.setThetaInterpolation(
                    RobotMovement.ThetaInterpolation.HYBRID
                );
                robotMovement.setReversed(true);
                currentPath = first;
                robotMovement.reset();
                setPathState(PathState.SECOND);
                break;

            case SECOND:
                if (robotMovement.getRealTranslationalEndDistance() <= 4) {
                    Pose p = robotMovement.getCurrentPose();
                    second[0] = new Pose(p.getX(), p.getY(), p.getTheta());
                    currentPath = second;
                    robotMovement.reset();

                    RobotConstants.setMinRadiusRange(18);
                    RobotConstants.setMaxRadiusRange(20);
                    setPathState(PathState.STOP);
                }
                break;

            case STOP:
                if (timer.getElapsedTimeSeconds() >= 10) {
                    setPathState(PathState.patata);
                }
                break;
        }
    }

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry);
        robotMovement = new RobotMovement(robotMap, startingPose);
        timer = new Timer();

        /*-- Drive Constants --*/
        driveConstants = new DriveConstants();

        driveConstants.frontLeftInverted = true;
        driveConstants.frontRightInverted = true;
        driveConstants.rearRightInverted = true;
        driveConstants.rearLeftInverted = true;

        driveConstants.WHEEL_RADIUS = 1; // inch
        driveConstants.GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        driveConstants.TRACK_WIDTH = 10; // in

        driveConstants.MAX_VEL = 90;
        driveConstants.MAX_ACCEL = 90;
        driveConstants.MAX_ANG_VEL = Math.toRadians(360);
        driveConstants.MAX_ANG_ACCEL = Math.toRadians(360);

        driveConstants.RUN_USING_ENCODER = false;

        driveConstants.frontLeftFeedForward[0] = 0;
        driveConstants.frontLeftFeedForward[1] = 1;
        driveConstants.frontLeftFeedForward[2] = 0;
        driveConstants.frontRightFeedForward[0] = 0;
        driveConstants.frontRightFeedForward[1] = 1;
        driveConstants.frontRightFeedForward[2] = 0;
        driveConstants.rearLeftFeedForward[0] = 0;
        driveConstants.rearLeftFeedForward[1] = 1;
        driveConstants.rearLeftFeedForward[2] = 0;
        driveConstants.rearRightFeedForward[0] = 0;
        driveConstants.rearRightFeedForward[1] = 1;
        driveConstants.rearRightFeedForward[2] = 0;

        driveConstants.VELO_KP = 0;
        driveConstants.VELO_KI = 0;
        driveConstants.VELO_KD = 0;

        driveConstants.TICKS_PER_REV = 537;
        driveConstants.MAX_RPM = 435;

        driveConstants.DEFAULT_SPEED_PERC = 1.0;
        driveConstants.SLOW_SPEED_PERC = 0.7;

        //- Default Path
        setPathState(PathState.FIRST);
        robot.setAutoEnabled(true);
    }

    public void initAllianceRelated(DecodeRobot.Alliance alliance) {
        robot = new DecodeRobot(
            robotMap,
            driveConstants,
            alliance
        );
    }

    @Override
    public void run() {
        super.run();
        autoPathUpdate();
        robotMovement.followPathUpdate(currentPath);
        robot.drive_update(robotMovement.getPowers());
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}
