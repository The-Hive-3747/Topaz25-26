package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back blue auto TWENTY ONE / shoots far", group = "default")
public class BackBlueAuto21 extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsBlue();
        startAtBack();
        setTurretFixedFar();
        setHoodPosFar();
        turnFlywheelShootNMove();
        shootAllThreeFarInHalves(0);
        intakeHP(0);
        shootAllThreeFarInHalves(0);
        intake3(0);
        shootAllThreeFarInHalves(0);
        intakeRecycled(0);
        shootAllThreeFarInHalves(0);
        intakeRecycled(0);
        shootAllThreeFarInHalves(0);
        intakeRecycled(0);
        shootAllThreeFarInHalves(0);
        intakeRecycled(0);
        shootAllThreeFarInHalves(0);
        parkAtBack();
    }
}