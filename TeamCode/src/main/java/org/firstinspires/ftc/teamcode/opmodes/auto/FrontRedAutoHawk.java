package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / HAWK", group = "default")
public class FrontRedAutoHawk extends AutoTemplate {
    @Override
    public void initAuto() {
        //TURRET_ANGLE_ADJUST_DEG = 15;
        startAsRedSOTM();
        startAtFrontSOTM();
        setHoodPosClose();
        isShootNMove = true;
        turnFlywheelShootNMove();
        shootAllThreeAtClose(0);
        intake1(0.3);
        shootAllThreeAtClose(0);
        intake2(0.3);
        openGate(0.7);
        shootAllThreeAtCloseCurved(0);
        openGate(0.7);
        intakeRecycledClose(0.3);
        shootAllThreeAtClose(0);
        intakeRecycledClose(0.3);
        shootAllThreeAtClose(0);
        parkAtFront();
    }
}