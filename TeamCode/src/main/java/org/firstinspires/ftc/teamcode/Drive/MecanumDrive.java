package org.firstinspires.ftc.teamcode.Drive;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.hardware.motors.Motor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.Arrays;
import java.util.List;

@Config
public class MecanumDrive {
    static DriveConstants RobotConstants;
    static Telemetry telemetry;
    private MotorExEx frontLeft, frontRight, rearRight, rearLeft;
    private List<MotorExEx> motors;
    private com.arcrobotics.ftclib.drivebase.MecanumDrive drive;

    private boolean fieldCentricEnabled = true;

    private boolean autoEnabled = false;

    public MecanumDrive(
            RobotMap robotMap,
            DriveConstants robotConstants
    ) {
        this.RobotConstants = robotConstants;
        this.telemetry = robotMap.getTelemetry();

        this.frontLeft = robotMap.getFrontLeftMotor();
        this.frontRight = robotMap.getFrontRightMotor();
        this.rearLeft = robotMap.getRearLeftMotor();
        this.rearRight = robotMap.getRearRightMotor();

        motors = Arrays.asList(frontLeft, frontRight, rearLeft, rearRight);

        if (RobotConstants.RUN_USING_ENCODER) {
            setMode(MotorExEx.RunMode.VelocityControl);
            resetEncoders();
        }

        setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        if (RobotConstants.RUN_USING_ENCODER && RobotConstants.MOTOR_VELO_PID != null) {
            setPIDFCoefficients(RobotConstants.VELO_KP, RobotConstants.VELO_KI, RobotConstants.VELO_KD);

            frontLeft.setFeedforwardCoefficients(
                    RobotConstants.frontLeftFeedForward[0],
                    RobotConstants.frontLeftFeedForward[1],
                    RobotConstants.frontLeftFeedForward[2]
            );
            frontRight.setFeedforwardCoefficients(
                    RobotConstants.frontRightFeedForward[0],
                    RobotConstants.frontRightFeedForward[1],
                    RobotConstants.frontRightFeedForward[2]
            );
            rearLeft.setFeedforwardCoefficients(
                    RobotConstants.rearLeftFeedForward[0],
                    RobotConstants.rearLeftFeedForward[1],
                    RobotConstants.rearLeftFeedForward[2]
            );
            rearRight.setFeedforwardCoefficients(
                    RobotConstants.rearRightFeedForward[0],
                    RobotConstants.rearRightFeedForward[1],
                    RobotConstants.rearRightFeedForward[2]
            );
        }

        frontLeft.resetEncoder();
        frontRight.resetEncoder();
        rearLeft.resetEncoder();
        rearRight.resetEncoder();

        setMotorsInverted(RobotConstants.frontLeftInverted, RobotConstants.frontRightInverted, RobotConstants.rearRightInverted, RobotConstants.rearLeftInverted);
        drive = new com.arcrobotics.ftclib.drivebase.MecanumDrive(
                frontLeft, frontRight, rearLeft, rearRight
        );
    }

    public void drive(double strafeSpeed, double forwardSpeed, double turnSpeed, double heading, double slow_input)
    {
        if(!autoEnabled){
            drive.setMaxSpeed(RobotConstants.DEFAULT_SPEED_PERC - slow_input * RobotConstants.SLOW_SPEED_PERC);
            drive.driveFieldCentric(strafeSpeed, forwardSpeed, turnSpeed, fieldCentricEnabled ? heading : 0);
        } else {
            drive.setMaxSpeed(1);
            drive.driveRobotCentric(strafeSpeed,forwardSpeed, turnSpeed);
        }
    }

    public void setMotorsInverted(
            boolean leftFrontInverted, boolean rightFrontInverted,
            boolean rightRearInverted, boolean leftRearInverted
    )
    {
        frontLeft.setInverted(leftFrontInverted);
        rearLeft.setInverted(leftRearInverted);
        frontRight.setInverted(rightFrontInverted);
        rearRight.setInverted(rightRearInverted);
    }
    public void setMode(Motor.RunMode mode)
    {
        for (MotorExEx motor : motors)
            motor.setRunMode(mode);
    }

    public void resetEncoders() {
        for (MotorExEx motor : motors) motor.resetEncoder();
    }

    public void setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior zeroPowerBehavior)
    {
        for (MotorExEx motor : motors)
            motor.setZeroPowerBehavior(zeroPowerBehavior);
    }
    public void setPIDFCoefficients(double kP, double kI, double kD)
    {
        for (MotorExEx motor : motors) {
            motor.setIntegralBounds(RobotConstants.minIntegralBound, RobotConstants.maxIntegralBound);
            motor.setVeloCoefficients(kP, kI, kD);
        }
    }

    public MotorExEx[] getMotors() {
        return new MotorExEx[]{frontLeft, frontRight, rearLeft, rearRight};
    }

    public void toggleMode() {
        fieldCentricEnabled = !fieldCentricEnabled;
    }

    public void setFieldCentric() {
        fieldCentricEnabled = true;
    }
    public void setRobotCentric() {
        fieldCentricEnabled = false;
    }

    public void setAutoEnabled(boolean enabled) {
        autoEnabled = enabled;
    }
}