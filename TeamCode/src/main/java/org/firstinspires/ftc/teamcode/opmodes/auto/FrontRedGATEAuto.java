package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / open gate", group = "default")
public class FrontRedGATEAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtFront();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAllThreeAtClose(0);
        intake1(0.1);
        shootAllThreeAtClose(0);
        intake2(0.1);
        openGate(1);
        shootAllThreeAtCloseCurved(0);
        parkAtFront();
    }
}