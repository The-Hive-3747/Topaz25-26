package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back blue auto / rogue", group = "custom")
public class BackBlueAutoRogue extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsBlue();
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