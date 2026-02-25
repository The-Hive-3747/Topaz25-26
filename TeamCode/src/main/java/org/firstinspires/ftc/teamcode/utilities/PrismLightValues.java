package org.firstinspires.ftc.teamcode.utilities;


import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.hardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import java.util.Arrays;

public class PrismLightValues {
    GoBildaPrismDriver prism;

    PrismAnimations.AnimationType selectedAnimation = PrismAnimations.AnimationType.SOLID;

    int startPoint = 0; // the start LED for any configured animation
    int endPoint = 12; // the end LED for a configured animation
    int animationSelector = 1;
    int brightness = 50; // the brightness of configured animation
    int period = 1000; // the period of a configured animation
    float speed = 0.5F; // the speed of a configured animation

    Color[] singleFillColors = {
            Color.RED, Color.WHITE, Color.BLUE, Color.GREEN, Color.YELLOW
    };

    public String animationCursor(PrismAnimations.AnimationType animationType, int selector){
        if(animationType.AnimationTypeIndex == selector){
            selectedAnimation = animationType;
            return "<--";
        }else return "";
    }




    public void AllianceColor() {

    }
}
