package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back red auto / shoots far SOTM", group = "default")
public class BackRedAutoSOTM extends AutoTemplate {
    @Override
    public void initAuto() {
        TURRET_ANGLE_ADJUST_DEG=-2;
        startAsRedSOTM();
        startAtBackSOTM();
        setHoodPosFar();
        turnFlywheelShootNMove();
        shootAllThreeFarInHalves(0);
        intakeHP(0.3);
        shootAllThreeFarInHalves(0);
        intake3(0.1);
        shootAllThreeFarInHalves(0);
        intakeHP(0.3);
        shootAllThreeFarInHalves(0);
        intakeHP(0.3);
        shootAllThreeFarInHalves(0);
        intakeHP(0.3);
        shootAllThreeFarInHalves(0);
        parkAtBack();
    }
}