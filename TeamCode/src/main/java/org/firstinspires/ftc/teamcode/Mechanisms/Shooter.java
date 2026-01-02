package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCoeffs;
import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
public class Shooter extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private final MotorExEx wheel1, wheel2;
    private ServoImplEx hoodServo;
    private CRServoImplEx turretServoDriver, turretServoFollower;
    private AnalogInput turretServoPot;

    // ---------------------------------------- Constants --------------------------------------- //
    // Wheel
    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2300; // WHEEL_MAX_RPM/60.0 * 28

    // Hood
    private static final double MIN_HOOD_POS = 0.92, MAX_HOOD_POS = 0.38;

    // Turret
    private static final double TURRET_RATIO = 1.0; // 12:1
    private static final double TURRET_MULTIPLIER = 1.0;

    // --------------------------- Turret Angle Calculation Variables --------------------------- //
    private double turns = 0, prevRawTurretServoAngle = 0;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean turretLockEnabled = false;
    private boolean hoodLockEnabled = false;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose;
    private final Pose REDGoalPose = new Pose(69, -69, 0); // TODO: Recheck
    private final Pose BLUEGoalPose = new Pose(69, 69, 0); // TODO: Recheck
    private final Pose goalPose;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private InterpLUT wheelSpeed, hoodAngle;
    private PIDFEx turretController, veloController; // TODO: Motion Profiling!!! Minimize
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, DecodeRobot.Alliance alliance,
                   Telemetry telemetry) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
        this.wheel2 = robotMap.getShooterWheel2Motor();
        this.hoodServo = robotMap.getHoodServo();
        this.turretServoDriver = robotMap.getTurretServoDriver();
        this.turretServoFollower = robotMap.getTurretServoFollower();
        this.turretServoPot = robotMap.getTurretServoPot();
//        this.telemetry = robotMap.getTelemetry();
        this.telemetry = telemetry;

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel2.setInverted(true);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        // TODO

        // Select Correct Goal Based On Alliance
        goalPose = (alliance == DecodeRobot.Alliance.RED) ? REDGoalPose : BLUEGoalPose;

        coeffsTurret = new PIDFExCoeffs(
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

        coeffsVelo = new PIDFExCoeffs(
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

        this.curPose = curPose;

        // Initialize LUTs here
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();

        // TODO: Populate LUTs with real data
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
        // ------------------------------------- Telemetry -------------------------------------- //
        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Turret Lock ", turretLockEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] Goal Dist: ", getDistanceToGoal());
        FtcDashboard.getInstance().getTelemetry().addData(
                "Actual Velo: ", wheel2.getVelocity()
        );
        FtcDashboard.getInstance().getTelemetry().addData(
                "Target Velo: ",
                wheelSpeed.get(getDistanceToGoal()) * 0.9 * MAX_TICKS_PER_S
        );

        // --------------------------------------- Turret --------------------------------------- //
        turretController.setSetPoint(Range.clip(getAngleToGoal(), -160, 10)); // TODO: Tune Clips
        setTurretPower(turretController.calculate(getTurretAngle()));

        if(getDistanceToGoal() < 31.9 || getDistanceToGoal() > 97.3) return; // TODO: LUT Bounds

        // ---------------------------------------- Hood ---------------------------------------- //
        hoodServo.setPosition(Range.scale(
                hoodAngle.get(getDistanceToGoal()), 0, 1, MIN_HOOD_POS, MAX_HOOD_POS)
        );

        // --------------------------------------- Wheels --------------------------------------- //
        wheel1.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
        wheel2.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
    }

    // ----------------------------------------- Wheels ----------------------------------------- //
    public double getControlledWheelPower(double power) {
        double speed = 0.9 * power * MAX_TICKS_PER_S;
        veloController.setSetPoint(speed);
        double velocity = veloController.calculate(wheel2.getCorrectedVelocity()) +
                feedforward.calculate(speed, wheel2.getAcceleration());
        return velocity / MAX_TICKS_PER_S;
    }

    public void enableWheels() {
        wheelsEnabled = true;
    }

    public void disableWheels() {
        wheelsEnabled = false;
        wheel1.set(0);
        wheel2.set(0);
    }

    public boolean areWheelsEnabled() {
        return wheelsEnabled;
    }

    // ----------------------------------------- Turret ----------------------------------------- //
//    public double getTurretAngle() {
//        return (((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION)*1.0;
//    }

    private double analogToDegrees(double analogVal, double maxVoltage) {
        return Range.scale(analogVal, 0.0, maxVoltage, 0, 360);
    }

    private double servoAngleToContinuousDegrees(double rawTurretServoAngle) {
        if(rawTurretServoAngle != prevRawTurretServoAngle) {
            if (Math.abs(rawTurretServoAngle - prevRawTurretServoAngle) >= 180)
                turns += (rawTurretServoAngle > prevRawTurretServoAngle) ? -1 : 1;

            prevRawTurretServoAngle = rawTurretServoAngle;
        }
        return rawTurretServoAngle + 360 * turns;
    }

    public double getTurretAngle() {
        return servoAngleToContinuousDegrees(
                analogToDegrees(
                        turretServoPot.getVoltage(), turretServoPot.getMaxVoltage()
                )
        )*TURRET_RATIO*TURRET_MULTIPLIER;
    }

    private void setTurretPower(double power) {
        turretServoDriver.setPower(power);
        turretServoFollower.setPower(-power);
    }

    // ---------------------------------------- IK Stuff ---------------------------------------- //
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
