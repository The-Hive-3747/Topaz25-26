package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
@Autonomous(name = "back red auto / shoots far", group = "default")
public class BackRedAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtBack();
        setTurretFixedFar();
        setHoodPosFar();
        turnFlywheelShootNMove();
        shootAllThreeFarInHalves(0);
        intakeHP(0.3);
        shootAllThreeFarInHalves(0);
        intake3(0.1);
        shootAllThreeFarInHalves(0);
        intakeRecycled(0.3);
        shootAllThreeFarInHalves(0);
        intakeRecycled(0.3);
        shootAllThreeFarInHalves(0);
        intakeRecycled(0.3);
        shootAllThreeFarInHalves(0);
        parkAtBack();
    }
}