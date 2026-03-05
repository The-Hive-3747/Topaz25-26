package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto")
public class FrontRedAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
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