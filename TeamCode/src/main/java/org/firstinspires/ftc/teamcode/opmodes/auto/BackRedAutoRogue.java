package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back red auto / rogue", group = "custom")
public class BackRedAutoRogue extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtBack();
        setTurretFixedFar();
        setHoodPosFar();
        turnFlywheelOnForBack();
        shootAllThreeAtFar(1.2);
        delay(0.6);
        intakeHP(0.75);
        shootAllThreeAtFar(0.8);
        delay(0.6);
        intake3(0.5);
        shootAllThreeAtFar(0.8);
        delay(0.6);
        parkAtBack();
    }
}