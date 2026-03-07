package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / perfect paradox", group = "custom")
public class FrontRedAutoPerfectParadox extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtFront();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAllThreeAtClose(0.4);
        intake1(0.5);
        openGate(0.2);
        shootAllThreeAtClose(0.6);
        intake2(0.5);
        shootAllThreeAtClose(0.6);
        parkAtFront();
    }
}