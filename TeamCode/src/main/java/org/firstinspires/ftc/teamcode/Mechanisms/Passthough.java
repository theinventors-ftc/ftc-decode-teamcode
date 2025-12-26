package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.Map;

public class Passthough extends SubsystemBase {
    private final ServoImplEx fingerF, fingerC, fingerR; // F: Front, C: Center, R: Rear
    private ColorSensor colorSensorF, colorSensorC, colorSensorR;

    private final ServoImplEx[] fingers;
    private final ColorSensor[] colorSensors;

    public enum FingerState {
        INTAKE,
        TOUCH,
        FEED;

        double[][] positions = {
                {0.0, 0.5, 1.0}, // FRONT
                {0.0, 0.5, 1.0}, // CENTER
                {0.0, 0.5, 1.0}  // REAR
        };

        public double getPosition(int idx) {
            return positions[idx][this.ordinal()];
        }
    }

    private FingerState[] states = {
            FingerState.INTAKE,
            FingerState.INTAKE,
            FingerState.INTAKE
    };

    public enum Color {
        PURPLE,
        GREEN,
        NONE
    }

    private Map<Color, Double[]> target_colors = Map.of(
            Color.PURPLE, new Double[]{0.0, 0.0, 0.0},
            Color.GREEN, new Double[]{0.0, 0.0, 0.0}
    );

    private Color[] current_colors = {
            Color.NONE,
            Color.NONE,
            Color.NONE
    };

    private MotifStorage.MotifState motif;

    private Telemetry telemetry;

    public Passthough(RobotMap robotMap, MotifStorage.MotifState motif) {
        this.motif = motif;

        this.fingerF = robotMap.getFingerFrontServo();
        this.fingerC = robotMap.getFingerCenterServo();
        this.fingerR = robotMap.getFingerRearServo();
        fingers = new ServoImplEx[]{fingerF, fingerC, fingerR};

        this.colorSensorF = robotMap.getColorSensorFront();
        this.colorSensorC = robotMap.getColorSensorCenter();
        this.colorSensorR = robotMap.getColorSensorRear();
        colorSensors = new ColorSensor[]{colorSensorF, colorSensorC, colorSensorR};

        this.telemetry = robotMap.getTelemetry();
    }

    @Override
    public void periodic() {
        telemetry.addData("[Passthough] FingerF State: ", getState(0));
        telemetry.addData("[Passthough] FingerC State: ", getState(1));
        telemetry.addData("[Passthough] FingerR State: ", getState(1));
    }

    public void setState(int finger, FingerState state) {
        if(state == getState(finger)) return;
        fingers[finger].setPosition(state.getPosition(finger));
    }

    public FingerState getState(int finger) {
        return states[finger];
    }

    // ------------------------------------- Color Sensors -------------------------------------- //

    private Color detectColor(int finger) {
        double[] colors = colorSensors[finger].getNormalizedColors();

        double minDistance = Double.MAX_VALUE;
        Color detectedColor = null;

        for(Map.Entry<Color, Double[]> entry : target_colors.entrySet()) {
            Double[] target = entry.getValue();

            double distance = Math.sqrt(
                    Math.pow(colors[0] - target[0], 2) +
                    Math.pow(colors[1] - target[1], 2) +
                    Math.pow(colors[2] - target[2], 2)
            );

            if(distance < minDistance) {
                minDistance = distance;
                detectedColor = entry.getKey();
            }
        }

        return detectedColor;
    }

    public void updateCurrentColors() {
        current_colors[0] = detectColor(0);
        current_colors[1] = detectColor(1);
        current_colors[2] = detectColor(2);
    }
}
