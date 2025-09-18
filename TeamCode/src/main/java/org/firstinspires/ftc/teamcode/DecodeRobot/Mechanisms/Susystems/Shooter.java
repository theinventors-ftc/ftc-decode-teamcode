package org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Controllers.PIDFEx;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;

@Config
public class Shooter extends SubsystemBase {
    private MotorExEx leftWheel, rightWheel;

    public static double KPL = 0.0, KIL = 0.0, KDL = 0.0, KVL = 1.21,
            KPR = 0.0, KIR = 0.0, KDR = 0.0, KVR = 1.1835, vel = 0.0;

    public static double shootingSpeed = 0.73;
    public static int vel_thresh = 1800;

    private PIDFEx left_controller, right_controller;

    public Shooter(HardwareMap hm) {
        this.leftWheel = new MotorExEx(hm, "leftShooter", Motor.GoBILDA.BARE);
        this.rightWheel = new MotorExEx(hm, "rightShooter", Motor.GoBILDA.BARE);
        leftWheel.setInverted(true);
        leftWheel.setRunMode(Motor.RunMode.RawPower);
        rightWheel.setRunMode(Motor.RunMode.RawPower);
        leftWheel.setFeedforwardCoefficients(0, KVL);
        rightWheel.setFeedforwardCoefficients(0, KVR);
        leftWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);
        rightWheel.setZeroPowerBehavior(Motor.ZeroPowerBehavior.FLOAT);

//        left_controller = new PIDFEx(KPL, KIL, KDL, 0, 0, 5, 500, 0.5);
//        right_controller = new PIDFEx(KPL, KIL, KDL, 0, 0, 5, 500, 0.5);
    }

    @Override
    public void periodic() {
        FtcDashboard.getInstance().getTelemetry().addData("LEFT VEL: ", leftWheel.getVelocity());
        FtcDashboard.getInstance().getTelemetry().addData("RIGHT VEL: ", rightWheel.getVelocity());
//        FtcDashboard.getInstance().getTelemetry().addData("Target VEL: ", vel);

//        left_controller.setSetPoint(vel);
//        right_controller.setSetPoint(vel);
//        left_controller.setPIDF(KPL, KIL, KDL, 0);
//        right_controller.setPIDF(KPR, KIR, KDR, 0);
//        leftWheel.set(left_controller.calculate(leftWheel.getVelocity()));
//        rightWheel.set(right_controller.calculate(rightWheel.getVelocity()));
    }

    public void rev() {
        this.set(shootingSpeed);
    }

    public void stop() {
        this.set(0.0);
    }

    public void intake() {
        this.set(-0.1);
    }

    public void set(double power) {
        leftWheel.set(power);
        rightWheel.set(power);
    }

    public boolean ready() {
        return leftWheel.getVelocity() > vel_thresh && rightWheel.getVelocity() > vel_thresh;
    }
}
