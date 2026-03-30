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

        beginPathBuilding();

        shootAtClose();
        intake1();
        shootAtClose();
        intake2();
        shootAtCloseCurved();
        intake3();
        shootAtClose();
        parkAtFront();
    }
}