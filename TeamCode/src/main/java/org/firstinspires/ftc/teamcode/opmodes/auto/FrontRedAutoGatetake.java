package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / GATETAKE", group = "default")
public class FrontRedAutoGatetake extends AutoTemplate {
    @Override
    public void initAuto() {
        //TURRET_ANGLE_ADJUST_DEG = 15;
        startAsRedSOTM();
        startAtFrontSOTM();
        setHoodPosClose();
        isShootNMove = true;
        turnFlywheelShootNMove();
        shootAllThreeAtClose(0);
        intake2(0.3);
        shootAllThreeAtClose(0);
        //intakeGate(0.3);
        parkAtFront();
    }
}