package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.GoBildaPinpointDriver;
import org.jetbrains.annotations.NotNull;

import java.lang.reflect.Modifier;

public class RobotMap {
    private GamepadExEx driverOp, toolOp;
    private MotorExEx frontLeft, rearLeft, frontRight, rearRight;
//    private IMU imu;
    private GoBildaPinpointDriver odo;
    private GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection, forwardEncoderDirection;
    private HardwareMap hm;
    private Telemetry telemetry;
    private GoBildaPinpointDriver.GoBildaOdometryPods encoderRes;

    public RobotMap(HardwareMap hm, Telemetry telemetry) {
        this(hm, telemetry, null, null);

    }

    public RobotMap (HardwareMap hm, Telemetry telemetry, Gamepad driverOp,
                     Gamepad toolOp) {
        this.telemetry = telemetry;
        this.hm = hm;
        this.driverOp = new GamepadExEx(driverOp);
        this.toolOp = new GamepadExEx(toolOp);

        /*--Motors--*/
        frontLeft = new MotorExEx(hm, "frontLeft", Motor.GoBILDA.RPM_435);
        rearLeft = new MotorExEx(hm, "rearLeft", Motor.GoBILDA.RPM_435);
        frontRight = new MotorExEx(hm, "frontRight", Motor.GoBILDA.RPM_435);
        rearRight = new MotorExEx(hm, "rearRight", Motor.GoBILDA.RPM_435);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        rearLeft.setRunMode(Motor.RunMode.RawPower);
        rearRight.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

        rearLeft.setInverted(true);
        frontLeft.setInverted(true);

//        /*--IMU--*/
//        imu = hm.get(IMU .class, "external_imu");
//        IMU.Parameters imuParameters = new IMU.Parameters(
//            new RevHubOrientationOnRobot(
//                RevHubOrientationOnRobot.LogoFacingDirection.BACKWARD,
//                RevHubOrientationOnRobot.UsbFacingDirection.DOWN
//            )
//        );
//        imu.initialize(imuParameters);

        /*--Encoders--*/
        odo = hm.get(GoBildaPinpointDriver.class, "odometry");
        encoderRes = GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
        forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;
        strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;

        /*--Util--*/
        for (LynxModule module : hm.getAll(LynxModule.class)) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }
    }

    /*--Gamepads--*/
    public GamepadExEx getDriverOp() {
        return driverOp;
    }

    public GamepadExEx getToolOp() {
        return toolOp;
    }

    /*--Motors--*/
    public MotorExEx getFrontLeftMotor() {
        return frontLeft;
    }

    public MotorExEx getFrontRightMotor() {
        return frontRight;
    }

    public MotorExEx getRearLeftMotor() {
        return rearLeft;
    }

    public MotorExEx getRearRightMotor() {
        return rearRight;
    }

    /*--Encoders--*/
    public GoBildaPinpointDriver.GoBildaOdometryPods getEncoderRes() {
        return encoderRes;
    }

    public GoBildaPinpointDriver getOdometry() {
        return odo;
    }

    public GoBildaPinpointDriver.EncoderDirection getStrafeEncoderDirection() {
        return strafeEncoderDirection;
    }

    public GoBildaPinpointDriver.EncoderDirection getForwardEncoderDirection() {
        return forwardEncoderDirection;
    }

    /*--Util--*/
    public Telemetry getTelemetry() {
        return telemetry;
    }

    /*--IMU--*/
    public IMU getIMU() {
        return null;
    }

    public HardwareMap getHm() {
        return hm;
    }

}
