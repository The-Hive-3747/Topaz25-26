package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name = "front blue auto / cubed ets", group = "custom")
public class FrontBlueAutoCubedETs extends AutoTemplate{
    @Override
    public void initAuto() {
        startAsBlue();
        startAtFront();
        setTurretFixedClose();
        setHoodPosClose();
        turnFlywheelOnForFront();
        shootAllThreeAtClose(0.3);
        intake1(0.5);
        openGate(0.2);
        shootAllThreeAtClose(0.5);
        intake2(0.5);
        shootAllThreeAtClose(0.5);
        intake3(0.5);
        shootAllThreeAtClose(0.5);
        parkAtFront();
    }
}
