package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name="limit switch")
@Disabled
public class LimitSwitchTele extends NextFTCOpMode {
    TouchSensor limitSwitch;
    boolean hasBeenPressed = false;

    @Override
    public void onInit() {
        limitSwitch = hardwareMap.get(TouchSensor.class, "limitSwitch");
    }


    @Override
    public void onUpdate() {
        if (!limitSwitch.isPressed()) {
            hasBeenPressed = true;
        }


        telemetry.addData("switch", limitSwitch.getValue());
        telemetry.addData("hasbeenpress", hasBeenPressed);
        telemetry.update();
    }

}


