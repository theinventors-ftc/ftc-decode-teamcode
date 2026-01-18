package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name = "TeleOP_BLUE", group = "")
public class TeleOpBLUE extends TeleOpBase {
    @Override
    public void initialize() {
        super.initialize();
        this.initAllianceRelated(DecodeRobot.Alliance.BLUE);
    }
}