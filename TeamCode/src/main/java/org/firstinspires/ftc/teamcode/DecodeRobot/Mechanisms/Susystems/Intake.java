package org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.Hardware.MotorExEx;

public class Intake extends SubsystemBase {
    private MotorExEx intakeMotor;

    public static double intake_speed = 0.7;

    public enum State {
        INTAKE,
        STOPPED,
        REVERSE
    }

    private State state = State.STOPPED;

    public Intake(HardwareMap hm) {
        this.intakeMotor = new MotorExEx(hm, "intake", Motor.GoBILDA.RPM_1150);
    }

    public void intake() {
        state = State.INTAKE;
        intakeMotor.set(intake_speed);
    }

    public void stop() {
        state = State.STOPPED;
        intakeMotor.set(0.0);
    }

    public void reverse() {
        state = State.REVERSE;
        intakeMotor.set(-intake_speed);
    }

    public State getState() {
        return state;
    }
}
