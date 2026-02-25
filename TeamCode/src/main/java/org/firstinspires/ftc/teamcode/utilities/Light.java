package org.firstinspires.ftc.teamcode.utilities;

import com.qualcomm.robotcore.hardware.Servo;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Light implements Component {
    Servo light = null;
    public static double COLOR_WHITE = 1.0;
    public static double COLOR_RED = 0.279;
    public static double COLOR_BLUE = 0.611;


    @Override
    public void postInit() {
        light = ActiveOpMode.hardwareMap().get(Servo.class, "light");
    }

    public void setColor(double color) {
        light.setPosition(color);
    }
}
