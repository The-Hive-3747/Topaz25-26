package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back red auto / shoots close", group = "default")
public class BackRedAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsRed();
        startAtBack();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAllThreeAtClose(0.3);
        intake1(0.3);
        openGate(0.2);
        shootAllThreeAtClose(0.4);
        intake2(0.3);
        shootAllThreeAtCloseCurved(0.4);
        intake3(0.3);
        shootAllThreeAtClose(0.4);
        parkAtFront();
    }
}