package org.firstinspires.ftc.teamcode.Mechanisms;

import com.arcrobotics.ftclib.command.SubsystemBase;
import com.qualcomm.robotcore.hardware.ServoImplEx;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.Hardware.ColorSensor;
import org.firstinspires.ftc.teamcode.MotifStorage;
import org.firstinspires.ftc.teamcode.RobotMap;

import java.util.Map;

public class Passthough extends SubsystemBase {
    // ---------------------------------------- Hardware ---------------------------------------- //
    private final ServoImplEx fingerF, fingerC, fingerR; // F: Front, C: Center, R: Rear
    private ColorSensor colorSensorF, colorSensorC, colorSensorR; // F: Front, C: Center, R: Rear

    private final ServoImplEx[] fingers;
    private final ColorSensor[] colorSensors;

    // ---------------------------------------- States ------------------------------------------ //

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

    // ----------------------------------------- Colors ----------------------------------------- //

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

    private static final Map<MotifStorage.MotifState, Color[]> MOTIF_MAP = Map.of(
            MotifStorage.MotifState.PPG, new Color[]{Color.PURPLE, Color.PURPLE, Color.GREEN},
            MotifStorage.MotifState.PGP, new Color[]{Color.PURPLE, Color.GREEN, Color.PURPLE},
            MotifStorage.MotifState.GPP, new Color[]{Color.GREEN, Color.PURPLE, Color.PURPLE}
    );

    private double distance_threshold = 40.0; // TODO: Chack if sensor hits a hole on the artifact

    // ----------------------------------------- Util ------------------------------------------- //

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
        telemetry.addData("[Passthough] ColorF: ", getCurrentColor(0));
        telemetry.addData("[Passthough] ColorC: ", getCurrentColor(1));
        telemetry.addData("[Passthough] ColorR: ", getCurrentColor(2));
    }

    // ------------------------------------- Finger Control ------------------------------------- //
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
        double distances = colorSensors[finger].getDistance(DistanceUnit.MM);

        double minColorDistance = Double.MAX_VALUE;
        Color detectedColor = Color.NONE;

        if(distances < distance_threshold) {
            for (Color color : Color.values()) {
                if (color == Color.NONE) continue;
                Double[] target = target_colors.get(color);
                double colorDistance = Math.sqrt(
                        Math.pow(colors[0] - target[0], 2) +
                        Math.pow(colors[1] - target[1], 2) +
                        Math.pow(colors[2] - target[2], 2)
                );
                if (colorDistance < minColorDistance) {
                    minColorDistance = colorDistance;
                    detectedColor = color;
                }
            }

            return detectedColor;
        }

        return detectedColor;
    }

    public void updateCurrentColors() { // Call only when needed, minimize I2C traffic
        current_colors[0] = detectColor(0);
        current_colors[1] = detectColor(1);
        current_colors[2] = detectColor(2);
    }

    public Color getCurrentColor(int finger) {
        return current_colors[finger];
    }

    public int[] shootingOrderMotif() {
        int[] order = {-1, -1, -1};

        int purple_count = 0, green_count = 0;
        for(Color color : current_colors) {
            if(color == Color.PURPLE) purple_count++;
            else if(color == Color.GREEN) green_count++;
        }

        if(purple_count != 2 || green_count != 1) return null;

        Color[] desired = MOTIF_MAP.get(motif);

        boolean[] used = new boolean[3];

        // Match desired colors to fingers
        for (int i = 0; i < 3; i++) {
            for (int finger = 0; finger < 3; finger++) {
                if (!used[finger] && current_colors[finger] == desired[i]) {
                    used[finger] = true;
                    order[i] = finger + 1; // convert to 1-based index
                    break;
                }
            }
        }

        return order;
    }
}
