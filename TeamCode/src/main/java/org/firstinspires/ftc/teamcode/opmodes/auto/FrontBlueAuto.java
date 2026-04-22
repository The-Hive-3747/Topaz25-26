package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front blue auto / shoots close", group = "default")
public class FrontBlueAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsBlue();
        startAtFront();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAllThreeAtClose(0);
        intake1(0.1);
        shootAllThreeAtClose(0);
        intake2(0.1);
        shootAllThreeAtCloseCurved(0); //SHOULD BE CURVED, BUT IT BROKE :( SO CHANGED BACK
        intake3(0.1);
        shootAllThreeAtClose(0);
        parkAtFront();
    }
}