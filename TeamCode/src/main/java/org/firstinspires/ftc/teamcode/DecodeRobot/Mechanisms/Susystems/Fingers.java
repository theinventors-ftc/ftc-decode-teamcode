package org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Fingers extends SubsystemBase {
    private ServoImplEx leftFinger, rightFinger;

    public enum Finger {
        LEFT(0),
        RIGHT(1);

        int idx;

        Finger(int idx) {
            this.idx = idx;
        }

        int getIdx() {
            return idx;
        }
    }

    public enum State {
        PARK(0),
        INTAKE(1),
        SHOOT(2);

        int idx;

        State(int idx) {
            this.idx = idx;
        }

        double pos[][] = {
                {0.3, 0.1, 0.5},
                {0.72, 0.9, 0.52}
        };

        public double getPos(Finger finger) {
            return pos[finger.getIdx()][idx];
        }
    }

    private State state = State.PARK;

    private Telemetry telemetry;

    public Fingers(HardwareMap hm, Telemetry telemetry) {
        this.leftFinger = hm.get(ServoImplEx.class, "leftFinger");
        this.rightFinger = hm.get(ServoImplEx.class, "rightFinger");

        this.telemetry = telemetry;

        move(State.INTAKE);
    }

    @Override
    public void periodic() {
        telemetry.addData("Left Finger: ", state.getPos(Finger.LEFT));
        telemetry.addData("Right Finger: ", state.getPos(Finger.RIGHT));
    }

    public void move(State state) {
        this.state = state;
        leftFinger.setPosition(state.getPos(Finger.LEFT));
        rightFinger.setPosition(state.getPos(Finger.RIGHT));
    }
}
