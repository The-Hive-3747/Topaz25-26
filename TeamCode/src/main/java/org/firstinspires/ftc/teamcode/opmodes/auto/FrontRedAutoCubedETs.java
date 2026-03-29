package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front red auto / cubed ets", group = "custom")
public class FrontRedAutoCubedETs extends AutoTemplate{
    @Override
    public void initAuto() {
        startAsRed();
        startAtFront();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAllThreeAtClose(0.5);
        intake1(0.4);
        openGate(0.5);
        shootAllThreeAtClose(0.5);
        intake2(0.2);
        shootAllThreeAtClose(0.5);
        intake3(0.2);
        shootAllThreeAtClose(0.5);
        parkAtFront();
    }
}
