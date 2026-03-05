package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front blue auto")
public class FrontBlueAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsBlue();
        startAtFront();
        shootAllThreeAtFront(1);
        intake1(0.5);
        shootAllThreeAtFront(0);
        intake2(0.5);
        shootAllThreeAtFront(0);
        intake3(0.5);
        shootAllThreeAtFront(0);
        parkAtFront();
    }
}