package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back blue auto / shoots close", group = "default")
public class BackBlueAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsBlue();
        startAtBack();
        setTurretFixedClose();
        shootAllThreeAtClose(1);
        intake1(0.5);
        shootAllThreeAtClose(0);
        intake2(0.5);
        shootAllThreeAtClose(0);
        intake3(0.5);
        shootAllThreeAtClose(0);
        parkAtFront();
    }
}