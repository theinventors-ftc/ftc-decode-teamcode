package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
public class Intake extends SubsystemBase {
    private final MotorExEx frontMotor, rearMotor;

    private final double INTAKE_POWER = 1.0;

    public enum IntakeState {
        INTAKE,
        REVERSE,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;
    private Telemetry telemetry;

    public Intake(RobotMap robotMap) {
        this.frontMotor = robotMap.getIntakeFrontMotor();
        this.rearMotor = robotMap.getIntakeRearMotor();
        frontMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);
        rearMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        telemetry.addData("[Intake] State ", state);
    }

    public void intake() {
        state = IntakeState.INTAKE;
        frontMotor.set(INTAKE_POWER);
        rearMotor.set(INTAKE_POWER);
    }

    public void reverse() {
        state = IntakeState.REVERSE;
        frontMotor.set(-INTAKE_POWER);
        rearMotor.set(-INTAKE_POWER);
    }

    public void stop() {
        state = IntakeState.STOPPED;
        frontMotor.set(0);
        rearMotor.set(0);
    }

    public IntakeState getState() {
        return state;
    }
}
