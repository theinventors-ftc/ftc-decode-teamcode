package org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Commands;

import com.arcrobotics.ftclib.command.CommandBase;

import org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems.Fingers;
import org.firstinspires.ftc.teamcode.DecodeRobot.Mechanisms.Susystems.Shooter;
import org.firstinspires.ftc.teamcode.Util.Timer;

public class ShootingCommand extends CommandBase {
    private Shooter shooter;
    private Fingers fingers;
    private Timer timer;
    private boolean start = true;

    public ShootingCommand(Shooter shooter, Fingers fingers) {
        this.shooter = shooter;
        this.fingers = fingers;
        timer = new Timer();

        addRequirements(this.shooter, this.fingers);
    }

    @Override
    public void initialize() {
        fingers.move(Fingers.State.PARK);
        shooter.rev();
    }

    @Override
    public void execute() {
        if(!shooter.ready()) return;
        if(start) {
            start = false;
            timer.resetTimer();
            fingers.move(Fingers.State.SHOOT);
        }
    }

    @Override
    public void end(boolean interrupted) {
        fingers.move(Fingers.State.INTAKE);
        shooter.stop();
        start = true;
    }

    @Override
    public boolean isFinished() {
        return !start && timer.getElapsedTimeSeconds() > 0.6;
    }
}
