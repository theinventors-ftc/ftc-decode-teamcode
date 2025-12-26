package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.IMU;
import com.qualcomm.robotcore.hardware.ServoImplEx;
import com.qualcomm.robotcore.hardware.configuration.ServoFlavor;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.Hardware.GamepadExEx;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.HardwareRelated.Localization.GoBildaPinpointDriver;

import java.util.List;

public class RobotMap {
    private GamepadExEx driverOp, toolOp;
    private MotorExEx frontLeft, rearLeft, frontRight, rearRight;
//    private IMU imu;
    private GoBildaPinpointDriver odo;
    private GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection, forwardEncoderDirection;
    private HardwareMap hm;
    private List<LynxModule> hubs;
    private Telemetry telemetry;
    private GoBildaPinpointDriver.GoBildaOdometryPods encoderRes;

    //// Mechanisms
    //Intake
    private MotorExEx intakeF, intakeR;
    private ServoImplEx armF, armR, passServo;

    // Passthough
    ServoImplEx fingerF, fingerC, fingerR;
    ColorSensor colorSensorF, colorSensorC, colorSensorR;

    // Shooter
    MotorExEx wheel1, wheel2, turretMotor;
    ServoImplEx hoodServo;


    public RobotMap(HardwareMap hm, Telemetry telemetry) {
        this(hm, telemetry, null, null);
    }

    public RobotMap (HardwareMap hm, Telemetry telemetry, Gamepad driverOp,
                     Gamepad toolOp) {
        this.telemetry = telemetry;
        this.hm = hm;

        if(driverOp != null) this.driverOp = new GamepadExEx(driverOp);
        if(toolOp != null) this.toolOp = new GamepadExEx(toolOp);

        hubs = hm.getAll(LynxModule.class);

        /*--Motors--*/
        frontLeft = new MotorExEx(hm, "frontLeft", Motor.GoBILDA.RPM_312);
        rearLeft = new MotorExEx(hm, "rearLeft", Motor.GoBILDA.RPM_312);
        frontRight = new MotorExEx(hm, "frontRight", Motor.GoBILDA.RPM_312);
        rearRight = new MotorExEx(hm, "rearRight", Motor.GoBILDA.RPM_312);

        frontLeft.setRunMode(Motor.RunMode.RawPower);
        rearLeft.setRunMode(Motor.RunMode.RawPower);
        rearRight.setRunMode(Motor.RunMode.RawPower);
        frontRight.setRunMode(Motor.RunMode.RawPower);

        frontLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearLeft.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        rearRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(Motor.ZeroPowerBehavior.BRAKE);

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
        forwardEncoderDirection = GoBildaPinpointDriver.EncoderDirection.REVERSED;
        strafeEncoderDirection = GoBildaPinpointDriver.EncoderDirection.FORWARD;

        /*--Util--*/
        for (LynxModule module : hubs) {
            module.setBulkCachingMode(LynxModule.BulkCachingMode.MANUAL);
        }

        //// ----------------------------------- Mechanisms ----------------------------------- ////
        // Intake
        intakeF = new MotorExEx(hm, "intake", Motor.GoBILDA.RPM_1150);
//        intakeR = new MotorExEx(hm, "intakeR", Motor.GoBILDA.RPM_1150);
        armF = hm.get(ServoImplEx.class, "arm");
        passServo = hm.get(ServoImplEx.class, "passthrough");
////        armR = hm.get(ServoImplEx.class, "armR");
//
//        // Passthrough
//        fingerF = hm.get(ServoImplEx.class, "fingerF");
//        fingerC = hm.get(ServoImplEx.class, "fingerC");
//        fingerR = hm.get(ServoImplEx.class, "fingerR");
//        colorSensorF = new ColorSensor(hm, "colorSensorF");
//        colorSensorC = new ColorSensor(hm, "colorSensorC");
//        colorSensorR = new ColorSensor(hm, "colorSensorR");

        // Shooter
        wheel1 = new MotorExEx(hm, "wheel1", Motor.GoBILDA.BARE);
        wheel2 = new MotorExEx(hm, "wheel2", Motor.GoBILDA.BARE);
        turretMotor = new MotorExEx(hm, "turretMotor", Motor.GoBILDA.RPM_312);
        hoodServo = hm.get(ServoImplEx.class, "hoodServo");
    }

    // ---------------------------------------- Gamepads ---------------------------------------- //
    public GamepadExEx getDriverOp() {
        return driverOp;
    }

    public GamepadExEx getToolOp() {
        return toolOp;
    }

    // ----------------------------------------- Motors ----------------------------------------- //
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

    // ---------------------------------------- Encoders ---------------------------------------- //
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

    // ------------------------------------------ Util ------------------------------------------ //
    public Telemetry getTelemetry() {
        return telemetry;
    }
    public List<LynxModule> getHubs() {
        return hubs;
    }

    // ------------------------------------------ IMU ------------------------------------------- //
    public IMU getIMU() {
        return null;
    }

    //// ------------------------------------- Mechanisms ------------------------------------- ////
    // Intake
    public MotorExEx getIntakeFrontMotor() {
        return intakeF;
    }

    public MotorExEx getIntakeRearMotor() {
        return intakeR;
    }

    public ServoImplEx getArm() {
        return armF;
    }

    public ServoImplEx getPassServo() {
        return passServo;
    }

    // Passthrough
    public ServoImplEx getFingerFrontServo() {
        return fingerF;
    }

    public ServoImplEx getFingerCenterServo() {
        return fingerC;
    }

    public ServoImplEx getFingerRearServo() {
        return fingerR;
    }

    public ColorSensor getColorSensorFront() {
        return colorSensorF;
    }

    public ColorSensor getColorSensorCenter() {
        return colorSensorC;
    }

    public ColorSensor getColorSensorRear() {
        return colorSensorR;
    }

    // Shooter
    public MotorExEx getShooterWheel1Motor() {
        return wheel1;
    }

    public MotorExEx getShooterWheel2Motor() {
        return wheel2;
    }

    public MotorExEx getTurretMotor() {
        return turretMotor;
    }

    public ServoImplEx getHoodServo() {
        return hoodServo;
    }
}
