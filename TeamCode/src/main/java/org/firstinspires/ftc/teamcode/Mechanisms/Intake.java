package org.firstinspires.ftc.teamcode.Mechanisms;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;
import org.firstinspires.ftc.teamcode.RobotMap;

@Config
public class Intake extends SubsystemBase {
    private final MotorExEx frontMotor;//, rearMotor;
    private final ServoImplEx arm, pass; //pass: 0, 0.15,

    private final double INTAKE_POWER = 1.0;

    public enum IntakeState {
        INTAKE,
        REVERSE,
        STOPPED
    }

    private IntakeState state = IntakeState.STOPPED;
    private Telemetry telemetry;

    public static double pos1 = 0.0, pos2 = 0.71;

    public Intake(RobotMap robotMap) {
        this.frontMotor = robotMap.getIntakeFrontMotor();
//        this.rearMotor = robotMap.getIntakeRearMotor();
        this.arm = robotMap.getArm();
        this.pass = robotMap.getPassServo();

        frontMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);
//        rearMotor.setZeroPowerBehavior(MotorExEx.ZeroPowerBehavior.BRAKE);

        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        telemetry.addData("[Intake] State ", state);

        pass.setPosition(pos1);
        arm.setPosition(pos2);
    }

    public void intake() {
        frontMotor.set(INTAKE_POWER);
//        rearMotor.set(INTAKE_POWER);
        state = IntakeState.INTAKE;
    }

    public void reverse() {
        frontMotor.set(-INTAKE_POWER);
//        rearMotor.set(-INTAKE_POWER);
        state = IntakeState.REVERSE;
    }

    public void stop() {
        frontMotor.set(0);
//        rearMotor.set(0);
        state = IntakeState.STOPPED;
    }

    public IntakeState getState() {
        return state;
    }
}
