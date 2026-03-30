package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "back blue auto / shoots close", group = "default")
public class BackBlueAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        startAsBlue();
        startAtBack();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAtClose();
        intake1();
        openGate();
        shootAtClose();
        intake2();
        shootAtCloseCurved();
        intake3();
        shootAtClose();
        parkAtFront();
    }
}