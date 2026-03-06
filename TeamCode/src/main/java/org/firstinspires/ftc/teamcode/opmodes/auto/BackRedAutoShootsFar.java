package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back red auto / shoots far", group = "default")
public class BackRedAutoShootsFar extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtBack();
        setTurretFixedFar();
        shootAllThreeAtFar(1);
        intake1(0.5);
        shootAllThreeAtFar(0);
        intake2(0.5);
        shootAllThreeAtFar(0);
        intake3(0.5);
        shootAllThreeAtFar(0);
        parkAtFront();
    }
}