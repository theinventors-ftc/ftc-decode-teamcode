package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.command.CommandOpMode;
import com.arcrobotics.ftclib.command.CommandScheduler;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.Drive.DecodeRobot;
import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;

//@Disabled
@TeleOp(name = "Do not run this TeleOP", group = "")
public class TeleOpBase extends CommandOpMode {
    GamepadExEx driverOp, toolOp;
    private DriveConstants RobotConstants;
    private ElapsedTime runtime;
    private DecodeRobot robot;

    private RobotMap robotMap;
    private Pose pose = new Pose(0,0,0);

    @Override
    public void initialize() {
        CommandScheduler.getInstance().reset();
        robotMap = new RobotMap(hardwareMap, telemetry, gamepad1, gamepad2);

        // ----------------------------------- Robot Constants ---------------------------------- //
        RobotConstants = new DriveConstants();

        RobotConstants.frontLeftInverted = true;
        RobotConstants.frontRightInverted = true;
        RobotConstants.rearRightInverted = true;
        RobotConstants.rearLeftInverted = true;

        RobotConstants.WHEEL_RADIUS = 1; // inch
        RobotConstants.GEAR_RATIO = 1; // output (wheel) speed / input (motor) speed
        RobotConstants.TRACK_WIDTH = 10; // in

        RobotConstants.MAX_VEL = 90;
        RobotConstants.MAX_ACCEL = 90;
        RobotConstants.MAX_ANG_VEL = Math.toRadians(360);
        RobotConstants.MAX_ANG_ACCEL = Math.toRadians(360);

        RobotConstants.RUN_USING_ENCODER = false;

        RobotConstants.frontLeftFeedForward[0] = 0;
        RobotConstants.frontLeftFeedForward[1] = 1;
        RobotConstants.frontLeftFeedForward[2] = 0;
        RobotConstants.frontRightFeedForward[0] = 0;
        RobotConstants.frontRightFeedForward[1] = 1;
        RobotConstants.frontRightFeedForward[2] = 0;
        RobotConstants.rearLeftFeedForward[0] = 0;
        RobotConstants.rearLeftFeedForward[1] = 1;
        RobotConstants.rearLeftFeedForward[2] = 0;
        RobotConstants.rearRightFeedForward[0] = 0;
        RobotConstants.rearRightFeedForward[1] = 1;
        RobotConstants.rearRightFeedForward[2] = 0;

        RobotConstants.VELO_KP = 0;
        RobotConstants.VELO_KI = 0;
        RobotConstants.VELO_KD = 0;

        RobotConstants.TICKS_PER_REV = 537;
        RobotConstants.MAX_RPM = 312;

        RobotConstants.DEFAULT_SPEED_PERC = 1.0;
        RobotConstants.SLOW_SPEED_PERC = 0.7;

        // ---------------------------- Transfer Pose from Autonomous --------------------------- //
        pose = PoseStorage.currentPose;

        initAllianceRelated(DecodeRobot.Alliance.BLUE);
    }

    public void initAllianceRelated(DecodeRobot.Alliance alliance) {
        robot = new DecodeRobot(
            robotMap,
            RobotConstants,
            alliance,
            pose
        );
    }

    @Override
    public void run() {
        super.run();
        robot.drive_update();
        telemetry.update();
        FtcDashboard.getInstance().getTelemetry().update();
    }
}