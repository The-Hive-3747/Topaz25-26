package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "back blue auto / disconnect", group = "custom")
public class BackBlueAutoDisconnect extends AutoTemplate {
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
        intakeHP(0.75);
        shootAllThreeAtFar(0.8);
        delay(0.6);
        parkAtBack();
    }
}