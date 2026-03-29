package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / shoots close", group = "default")
public class FrontRedAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtFront();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAllThreeAtClose(0.5);
        intake1(0.3);
        //openGate(0.2);
        shootAllThreeAtClose(0.45);
        intake2(0.15);
        shootAllThreeAtCloseCurved(0.35);
        intake3(0.15);
        shootAllThreeAtClose(0.45);
        parkAtFront();
    }
}