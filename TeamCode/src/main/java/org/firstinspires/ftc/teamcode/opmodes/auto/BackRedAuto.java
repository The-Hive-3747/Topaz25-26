package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back red auto / shoots far", group = "default")
public class BackRedAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtBack();
        setTurretFixedFar();
        setHoodPosFar();
        turnFlywheelOnForBack();
        waitUntilFlywheelAtSpeed(3);
        shootAllThreeAtFar(0);
        intakeHP(0.5);
        shootAllThreeAtFar(0);
        intake3(0.1);
        shootAllThreeAtFar(0);
        intake2(0.1);
        shootAllThreeAtFar(0);
        parkAtBack();
    }
}