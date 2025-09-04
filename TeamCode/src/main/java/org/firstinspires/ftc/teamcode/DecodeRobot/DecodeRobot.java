package org.firstinspires.ftc.teamcode.DecodeRobot;

import com.arcrobotics.ftclib.command.ConditionalCommand;
import com.arcrobotics.ftclib.command.InstantCommand;
import com.arcrobotics.ftclib.command.ParallelCommandGroup;
import com.arcrobotics.ftclib.command.SequentialCommandGroup;
import com.arcrobotics.ftclib.command.WaitCommand;
import com.arcrobotics.ftclib.command.WaitUntilCommand;
import com.arcrobotics.ftclib.command.button.Trigger;
import com.arcrobotics.ftclib.gamepad.GamepadKeys;

import org.firstinspires.ftc.teamcode.Drive.DriveConstants;
import org.firstinspires.ftc.teamcode.Hardware.RobotEx;
import org.firstinspires.ftc.teamcode.PurePursuit.Base.Coordination.Pose;
import org.firstinspires.ftc.teamcode.RobotMap;

public class DecodeRobot extends RobotEx {
    protected RobotMap robotMap;
    //--------------------------------- Initialize SubsyMechanisms -------------------------------//


    // ---------------------------------- Initialize Controllers -------------------------------- //
//    protected ForwardControllerSubsystem forwardController;
//    protected StrafeControllerSubsystem strafeControllerSubsystem;

    private boolean hasInit = false;

    public DecodeRobot(RobotMap robotMap, DriveConstants RobotConstants,
                            OpModeType opModeType, Alliance alliance, Pose startingPose) {
        super(robotMap, RobotConstants, opModeType, alliance, startingPose);
        this.robotMap = robotMap;

        new Trigger(() -> (Math.abs(drivetrainForward()) > 0.1 ||
                Math.abs(drivetrainStrafe()) > 0.1 ||
                Math.abs(drivetrainTurn()) > 0.1) && !hasInit)
                .whenActive(new InstantCommand(this::initMechanismsTeleOp));
    }

    @Override
    public double drivetrainForward() {
        return super.drivetrainForward();
    }

    @Override
    public double drivetrainStrafe() {
        return super.drivetrainStrafe();
    }

    @Override
    public double drivetrainTurn() { return super.drivetrainTurn(); }
}
