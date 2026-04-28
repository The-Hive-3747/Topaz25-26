package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / open gate", group = "default")
public class FrontRedGATEAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRedSOTM();
        startAtFrontSOTM();
        setHoodPosClose();
        isShootNMove = true;
        turnFlywheelShootNMove();
        shootAllThreeAtClose(0);
        intake1(0.3);
        shootAllThreeAtClose(0);
        intake2(0.3);
        openGate(1);
        shootAllThreeAtClose(0);
        intake3(0.3);
        shootAllThreeAtClose(0);
        parkAtFront();
    }
}