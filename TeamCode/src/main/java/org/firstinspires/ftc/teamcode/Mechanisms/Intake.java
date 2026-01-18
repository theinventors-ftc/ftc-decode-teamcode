package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Math.MathFunction;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.function.DoubleSupplier;

@Config
public class Intake extends SubsystemBase {
    private final MotorExEx frontMotor;//, rearMotor;

    private final double INTAKE_POWER = 1.0;

    private DoubleSupplier robotHeading, robotForwardPower, robotStrafePower;
    private double V, Vx;

    public enum IntakeState {
        INTAKE,
        REVERSE,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;
    private Telemetry telemetry;

    public Intake(RobotMap robotMap, DoubleSupplier robotHeading, DoubleSupplier robotForwardPower,
                  DoubleSupplier robotStrafePower) {
        this.frontMotor = robotMap.getIntakeFrontMotor();
//        this.rearMotor = robotMap.getIntakeRearMotor();
        frontMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
//        rearMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.FLOAT);
//        rearMotor.setInverted(true);

        this.robotHeading = robotHeading;
        this.robotForwardPower = robotForwardPower;
        this.robotStrafePower = robotStrafePower;

        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        V = Math.hypot(robotForwardPower.getAsDouble(), robotStrafePower.getAsDouble());
        Vx = Math.cos(Math.toRadians(robotHeading.getAsDouble())) * V;
        Vx = 0.0;

        telemetry.addData("[Intake] State ", state);
        telemetry.addData("[Intake] Forward: ", robotForwardPower.getAsDouble());
        telemetry.addData("[Intake] Theta: ", robotHeading.getAsDouble());
        telemetry.addData("[Intake] Vx: ", Vx);

        if(state == IntakeState.INTAKE) {
            frontMotor.set(Vx >= 0 ? INTAKE_POWER : 0);
//            rearMotor.set(Vx <= 0 ? INTAKE_POWER : 0);
        }
    }

    public void intake() {
        state = IntakeState.INTAKE;
    }

    public void reverse() {
        state = IntakeState.REVERSE;
        frontMotor.set(-INTAKE_POWER);
//        rearMotor.set(-INTAKE_POWER);
    }

    public void stop() {
        state = IntakeState.STOPPED;
        frontMotor.set(0);
//        rearMotor.set(0);
    }

    public IntakeState getState() {
        return state;
    }
}
