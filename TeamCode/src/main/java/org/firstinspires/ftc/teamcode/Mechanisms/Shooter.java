package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCon;
import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

@Config
public class Shooter extends SubsystemBase {
    private final MotorExEx wheel1, wheel2, turretMotor;
    private ServoImplEx hoodServo;

    private boolean wheelsEnabled = false;
    private boolean turretLockEnabled = false;
    private boolean hoodLockEnabled = false;

    private Supplier<Pose> curPose;
    private final Pose REDGoalPose = new Pose(69, -69, 0);
    private final Pose BLUEGoalPose = new Pose(69, 69, 0);
    private final Pose goalPose;

    private Telemetry telemetry;

    // Safety shooter positions ++ InterpolatedLUTs ++ Turret PID ++ Wheel Velo PID

    private InterpLUT wheelSpeed, hoodAngle;

    private final int MOTOR_TICKS_PER_REV = 538;
    private final double GEAR_RATIO = 130.0/28.0;
    private final int TICKS_PER_FULL_ROTATION = (int)(MOTOR_TICKS_PER_REV * GEAR_RATIO);

    private PIDFEx turretController, veloController; // TODO: Motion Profiling!!! Minimize
    private PIDFExCon coeffsTurret, coeffsVelo;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2300; // WHEEL_MAX_RPM/60.0 * 28

    private static final double MIN_HOOD = 0.92, MAX_HOOD = 0.38;

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, DecodeRobot.Alliance alliance, Telemetry telemetry) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setInverted(true);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);

        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.resetEncoder();
        turretMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);
        coeffsTurret = new PIDFExCon(
                0.045,
                0.0,
                0.005,
                0.0,
                0.2,
                0.5,
                0,
                0
        );
        turretController = new PIDFEx(coeffsTurret);

        coeffsVelo = new PIDFExCon(
                0.00085,
                0.0,
                0.0,
                0.0,
                0.0,
                10,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        this.hoodServo = robotMap.getHoodServo();

//        this.telemetry = robotMap.getTelemetry();
        this.telemetry = telemetry;
        this.curPose = curPose;

        // Select Correct Goal Based On Alliance
        goalPose = (alliance == DecodeRobot.Alliance.RED) ? REDGoalPose : BLUEGoalPose;

        // Initialize LUTs here
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();

        wheelSpeed.add(31.9, 0.74);
        wheelSpeed.add(38, 0.72);
        wheelSpeed.add(53.54, 0.76);
        wheelSpeed.add(67.41, 0.78);
        wheelSpeed.add(81.24, 0.84);
        wheelSpeed.add(97.3, 0.91);

        hoodAngle.add(31.9, 0.1);
        hoodAngle.add(38, 0.5);
        hoodAngle.add(53.54, 0.7);
        hoodAngle.add(67.41, 0.76);
        hoodAngle.add(81.24, 0.7);
        hoodAngle.add(97.3, 0.94);

        wheelSpeed.createLUT();
        hoodAngle.createLUT();
    }

    @Override
    public void periodic() {
        turretController.setSetPoint(Range.clip(getAngleToGoal(), -160, 10));
        turretMotor.set(turretController.calculate(getTurretAngle()));

        if(getDistanceToGoal() > 31.9 && getDistanceToGoal() < 97.3) {
            hoodServo.setPosition(Range.scale(hoodAngle.get(getDistanceToGoal()), 0, 1, MIN_HOOD, MAX_HOOD));
            wheel1.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
            wheel2.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
            FtcDashboard.getInstance().getTelemetry().addData("Target Velo: ", wheelSpeed.get(getDistanceToGoal()) * 0.9 * MAX_TICKS_PER_S);
        }

        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Turret Lock ", turretLockEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Pos: ", getTurretAngle());
        telemetry.addData("[Shooter] Goal Dist: ", getDistanceToGoal());
        FtcDashboard.getInstance().getTelemetry().addData("Actual Velo: ", wheel2.getVelocity());
    }

    public double getControlledWheelPower(double power) {
        double speed = 0.9 * power * MAX_TICKS_PER_S;
        veloController.setSetPoint(speed);
        double velocity = veloController.calculate(wheel2.getCorrectedVelocity()) + feedforward.calculate(speed, wheel2.getAcceleration());
        return velocity / MAX_TICKS_PER_S;
    }


    public void enableWheels() {
        wheel1.set(0.5);
        wheel2.set(0.5);
        wheelsEnabled = true;
    }

    public void disableWheels() {
        wheel1.set(0);
        wheel2.set(0);
        wheelsEnabled = false;
    }

    public boolean areWheelsEnabled() {
        return wheelsEnabled;
    }

    public double getTurretAngle() {
        return ((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION;
    }


    // IK Stuff
    private double getDistanceToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    public double getAngleToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.toDegrees(Math.atan2(dy, dx)) - MathFunction.wrapDegrees(pose.getTheta());
    }
}
