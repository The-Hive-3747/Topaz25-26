package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

@Disabled
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
        //intakeRecycledFar(0);
        shootAllThreeFarInHalves(0);
        //intakeRecycledFar(0);
        shootAllThreeFarInHalves(0);
        //intakeRecycledFar(0);
        shootAllThreeFarInHalves(0);
        //intakeRecycledFar(0);
        shootAllThreeFarInHalves(0);
        parkAtBack();
    }
}