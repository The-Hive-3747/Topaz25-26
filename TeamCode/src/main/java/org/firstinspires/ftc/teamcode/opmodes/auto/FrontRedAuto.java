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
        shootAllThreeAtClose(0);
        intake1(0.1);
        shootAllThreeAtClose(0);
        intake2(0.1);
        shootAllThreeAtCloseCurved(0);
        intake3(0.1);
        shootAllThreeAtClose(0);
        parkAtFront();
    }
}