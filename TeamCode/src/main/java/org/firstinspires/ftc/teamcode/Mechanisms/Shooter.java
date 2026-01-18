package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.controller.wpilibcontroller.SimpleMotorFeedforward;
import com.arcrobotics.ftclib.util.InterpLUT;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServoImplEx;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Supplier;
import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Controllers.PIDFExCoeffs;
import org.firstinspires.ftc.teamcode.Controllers.SigmoidPositionProfile;
import org.firstinspires.ftc.teamcode.Controllers.StateMachine;
import org.firstinspires.ftc.teamcode.DecodeRobot;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.RobotMap;
import org.firstinspires.ftc.teamcode.Util.Timer;

import java.util.function.DoubleSupplier;

@Config
public class Shooter extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private MotorExEx wheel1, wheel2;
    private ServoImplEx hoodServo;
    private MotorExEx turretMotor;

    // ---------------------------------------- Constants --------------------------------------- //
    // Wheel
    private static final int WHEEL_TICKS_PER_REV = 28, WHEEL_MAX_RPM = 5800;
    private static final double MAX_TICKS_PER_S = 2700; // WHEEL_MAX_RPM/60.0 * 28

    // Hood
    private static final double MIN_HOOD_POS = 0.95, MAX_HOOD_POS = 0.07;

    // Turret
    private static final double TICKS_PER_FULL_ROTATION = 1916.0;
    private static final double MAX_TURRET_POWER = 1.0;
    private static final double MIN_TURRET_ANGLE = -91, MAX_TURRET_ANGLE = 203.0;


    public static double velo = 0.0, hood = 0.5;

    // ----------------------------------------- States ----------------------------------------- //
    private boolean wheelsEnabled = false;
    private boolean turretLockEnabled = false;
    private boolean hoodLockEnabled = false;

    // ---------------------------------------- Poses ------------------------------------------- //
    private Supplier<Pose> curPose;
    private final Pose REDGoalPose = new Pose(67, -67, 0);
    private final Pose BLUEGoalPose = new Pose(67, 67, 0);
    private final Pose goalPose;

    // ---------------------------------- Controllers and LUTs ---------------------------------- //
    private InterpLUT wheelSpeed, hoodAngle;
    private PIDFEx turretController, veloController;
    private PIDFExCoeffs coeffsTurret, coeffsVelo;
    private SimpleMotorFeedforward feedforward = new SimpleMotorFeedforward(0, 1, 0);

    // ------------------------------------ Turret Zeroing -------------------------------------- //
    private boolean turretZeroed = false;
    private double turretZeroPower = -0.4;
    private double turretZeroCurrentThreshold = 3.6;
    private double turretZeroOffset = 93.52;
    private StateMachine hasStalled;

    // ------------------------------------------ Util ------------------------------------------ //
    private Telemetry telemetry;
    private DoubleSupplier voltage;

    public static double kp = 0.04, ki = 0.14, kd = 0.002, kf = 0.0;

    public Shooter(RobotMap robotMap, Supplier<Pose> curPose, DecodeRobot.Alliance alliance) {
        this.wheel1 = robotMap.getShooterWheel1Motor();
//        this.wheel2 = robotMap.getShooterWheel2Motor();
        this.wheel2 = robotMap.getIntakeRearMotor();
        this.hoodServo = robotMap.getHoodServo();
        this.turretMotor = robotMap.getTurretMotor();
        turretMotor.setInverted(true);
        turretMotor.resetEncoder();
        this.telemetry = robotMap.getTelemetry();

        hasStalled = new StateMachine(() -> ((DcMotorEx)turretMotor.getRawMotor()).getCurrent(CurrentUnit.AMPS) > turretZeroCurrentThreshold, 400);

        wheel1.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
        wheel1.setInverted(true);
        wheel2.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);

        // Select Correct Goal Based On Alliance
        goalPose = (alliance == DecodeRobot.Alliance.RED) ? REDGoalPose : BLUEGoalPose;

        coeffsTurret = new PIDFExCoeffs(
                0.08,
                0.14,
                0.0012,
                0.0,
                0.1,
                0.02,
                15,
                0.5
        );
        turretController = new PIDFEx(coeffsTurret);

        coeffsVelo = new PIDFExCoeffs(
                12,
                0.0,
                0.0,
                0.0,
                0.0,
                3,
                600,
                0.8
        );
        veloController = new PIDFEx(coeffsVelo);

        this.curPose = curPose;

        // Initialize LUTs here
        wheelSpeed = new InterpLUT();
        hoodAngle = new InterpLUT();

        wheelSpeed.add(32.7, 0.525);
        wheelSpeed.add(39.7, 0.56);
        wheelSpeed.add(46.69, 0.57);
        wheelSpeed.add(53, 0.6);
        wheelSpeed.add(61.5, 0.62);
        wheelSpeed.add(70.01, 0.648);
        wheelSpeed.add(80.46, 0.667);
        wheelSpeed.add(89.92, 0.68);
        wheelSpeed.add(101.81, 0.73);
        wheelSpeed.add(111.24, 0.75);
        wheelSpeed.add(120.18, 0.77);
        wheelSpeed.add(130.35, 0.8);
        wheelSpeed.add(142.32, 0.83);
        wheelSpeed.add(154.41, 0.88);
        wheelSpeed.add(164.78, 0.906);

        hoodAngle.add(32.7, 0.0);
        hoodAngle.add(39.7, 0.2);
        hoodAngle.add(46.69, 0.2);
        hoodAngle.add(53, 0.33);
        hoodAngle.add(61.5, 0.38);
        hoodAngle.add(70.01, 0.44);
        hoodAngle.add(80.46, 0.57);
        hoodAngle.add(89.92, 0.57);
        hoodAngle.add(101.81, 0.7);
        hoodAngle.add(111.24, 0.7);
        hoodAngle.add(120.18, 0.7);
        hoodAngle.add(130.35, 0.87);
        hoodAngle.add(142.32, 0.95);
        hoodAngle.add(154.41, 0.95);
        hoodAngle.add(164.78, 0.96);

        wheelSpeed.createLUT();
        hoodAngle.createLUT();

        voltage = () -> robotMap.getBattery().getVoltage();
    }

    @Override
    public void periodic() {
        if(!turretZeroed) {
            hasStalled.update();
            turretMotor.set(turretZeroPower);
            if(hasStalled.isJustActive()) {
                turretMotor.resetEncoder();
                turretMotor.set(0);
                turretZeroed = true;
            }
            return;
        }

        turretController.setP(kp);
        turretController.setI(ki);
        turretController.setD(kd);
        turretController.setF(kf);

        // ------------------------------------- Telemetry -------------------------------------- //
        telemetry.addData("[Shooter] Wheel State ", wheelsEnabled);
        telemetry.addData("[Shooter] Turret Lock ", turretLockEnabled);
        telemetry.addData("[Shooter] Hood Lock ", hoodLockEnabled);
        telemetry.addData("[Shooter] Turret Angle: ", getTurretAngle());
        telemetry.addData("[Shooter] Goal Dist: ", getDistanceToGoal());
        telemetry.addData("[Shooter] Goal Angle: ", getAngleToGoal());

        // --------------------------------------- Turret --------------------------------------- //
        turretController.setSetPoint(Range.clip(getAngleToGoal(), MIN_TURRET_ANGLE, MAX_TURRET_ANGLE));

        turretMotor.set(Range.clip(
                turretController.calculate(getTurretAngle()),
                -MAX_TURRET_POWER,
                MAX_TURRET_POWER
        ));

        if(!inLUTRange()) return;

        // ---------------------------------------- Hood ---------------------------------------- //
        hoodServo.setPosition(Range.scale(
                hoodAngle.get(getDistanceToGoal()), 0, 1, MIN_HOOD_POS, MAX_HOOD_POS)
        );

        // --------------------------------------- Wheels --------------------------------------- //
        if(wheelsEnabled) {
            wheel1.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
            wheel2.set(getControlledWheelPower(wheelSpeed.get(getDistanceToGoal())));
        }
    }

    // ----------------------------------------- Wheels ----------------------------------------- //
    public double getControlledWheelPower(double power) {
        double speed = 0.9 * power * MAX_TICKS_PER_S;
        veloController.setSetPoint(speed);
        double velocity = veloController.calculate(wheel1.getCorrectedVelocity()) +
                feedforward.calculate(speed, wheel1.getAcceleration());
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

    public boolean wheelsAtSpeed() {
        return Math.abs(veloController.getPositionError()) < 50;
    }

    // ----------------------------------------- Turret ----------------------------------------- //
    public double getTurretAngle() {
        return (((turretMotor.getCurrentPosition())%TICKS_PER_FULL_ROTATION)*360.0/TICKS_PER_FULL_ROTATION)*(180.0/181.4) - turretZeroOffset;
    }

    public boolean turretInRange() {
        double angleToGoal = getAngleToGoal();
        return angleToGoal > -90 && angleToGoal < 225;
    }

    public boolean turretAtGoal() {
        return Math.abs(turretController.getPositionError()) < 1.5;
    }

    // ---------------------------------------- IK Stuff ---------------------------------------- //
    private double getDistanceToGoal() {
        Pose pose = curPose.get();
        double dx = goalPose.getX() - pose.getX();
        double dy = goalPose.getY() - pose.getY();
        return Math.hypot(dx, dy);
    }

    public double getAngleToGoal() {
        double dx = goalPose.getX() - curPose.get().getX();
        double dy = goalPose.getY() - curPose.get().getY();

        double targetAngle = Math.toDegrees(Math.atan2(dy, dx));

        double robotHeading = curPose.get().getTheta() % 360;
        if (robotHeading >= 180) robotHeading -= 360;
        if (robotHeading < -180) robotHeading += 360;

        double relativeAngle = targetAngle - robotHeading;
        relativeAngle %= 360;
        if (relativeAngle >= 180) relativeAngle -= 360;
        if (relativeAngle < -180) relativeAngle += 360;

        if (relativeAngle < -95) relativeAngle = -95;
        if (relativeAngle > 205) relativeAngle = 205;

        return relativeAngle;
    }

    public boolean inLUTRange() {
        double dist = getDistanceToGoal();
        return dist > 32.8 && dist < 164.78;
    }
}
