package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / 15", group = "default")
public class FrontRedAutoCubedETs extends AutoTemplate {
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
        shootAtClose();
        intake3();
        shootAtFar();
        intakeHP();
        shootAtFar();
        parkAtFront();
    }
}