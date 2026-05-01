package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front blue auto / HAWK", group = "default")
public class FrontBlueAutoHawk extends AutoTemplate {
    @Override
    public void initAuto() {
        //TURRET_ANGLE_ADJUST_DEG = 15;
        startAsBlueSOTM();
        startAtFrontSOTM();
        setHoodPosClose();
        isShootNMove = true;
        turnFlywheelShootNMove();
        shootAllThreeAtClose(0);
        intake1(0.1);
        shootAllThreeAtClose(0);
        intake2(0.1);
        openGate(0.4);
        shootAllThreeAtCloseCurved(0);
        openGate(0.4);
        intakeRecycledClose(0.1);
        shootAllThreeAtClose(0);
        intakeRecycledClose(0.1);
        shootAllThreeAtClose(0);
        parkAtFront();
    }
}