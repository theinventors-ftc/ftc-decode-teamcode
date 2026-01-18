package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOP_RED", group = "")
public class TeleOpRED extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        this.initAllianceRelated(DecodeRobot.Alliance.RED);
    }
}