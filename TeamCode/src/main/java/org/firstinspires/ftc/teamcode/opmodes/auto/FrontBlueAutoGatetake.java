package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front blue auto / GATETAKE", group = "default")
public class FrontBlueAutoGatetake extends AutoTemplate {
    @Override
    public void initAuto() {
        //TURRET_ANGLE_ADJUST_DEG = 15;
        startAsBlueSOTM();
        startAtFrontSOTM();
        setHoodPosClose();
        isShootNMove = true;
        turnFlywheelShootNMove();
        shootAllThreeAtClose(0);
        //intakeGate(1);
        shootAllThreeAtClose(0);
        parkAtFront();
    }
}
