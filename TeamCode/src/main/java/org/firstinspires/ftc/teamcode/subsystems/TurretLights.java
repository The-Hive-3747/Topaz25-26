package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.Color;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utilities.PrismAnimations;

public class TurretLights {
    GoBildaPrismDriver prism;
    PrismAnimations.Solid red = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid blue = new PrismAnimations.Solid(Color.BLUE);
    PrismAnimations.Solid shootNow = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid green = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid noShoot = new PrismAnimations.Solid(Color.MAGENTA);
    PrismAnimations.Solid purple = new PrismAnimations.Solid(Color.MAGENTA);

    HardwareMap hardwareMap;
    Telemetry telemetry;
    int brightness = 30;
    int startIndex = 0;
    int stopIndex = 23;


    public TurretLights(HardwareMap hm, Telemetry tm) {
        hardwareMap = hm;
        telemetry = tm;
        prism = hardwareMap.get(GoBildaPrismDriver.class, "lights");

        prism.setStripLength(24);


    }

    public void readyToShoot() {
        shootNow.setStartIndex(startIndex);
        shootNow.setStopIndex(stopIndex);
        shootNow.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, shootNow);
    }

    public void notReadyToShoot() {
        noShoot.setStartIndex(startIndex);
        noShoot.setStartIndex(stopIndex);
        noShoot.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, noShoot);
    }

    public void redAlliance() {
        red.setBrightness(brightness);
        red.setStartIndex(startIndex);
        red.setStopIndex(stopIndex);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, red);
    }

    public void blueAlliance() {
        blue.setBrightness(brightness);
        blue.setStartIndex(startIndex);
        blue.setStopIndex(stopIndex);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blue);
    }

    public void gPP() {
        green.setBrightness(50);
        purple.setBrightness(50);
        green.setStartIndex(0);
        green.setStopIndex(1);
        purple.setStartIndex(0);
        purple.setStartIndex(5);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, purple);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, green);
    }

    public void pGP() {
        green.setBrightness(50);
        purple.setBrightness(50);
        purple.setStartIndex(0);
        purple.setStopIndex(5);
        green.setStartIndex(2);
        green.setStopIndex(3);
        //purple2.setStartIndex(4);
        //purple2.setStopIndex(5);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, purple);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, green);
    }
    public void pPG(){
        green.setBrightness(50);
        purple.setBrightness(50);
        purple.setStartIndex(0);
        purple.setStopIndex(3);
        green.setStartIndex(4);
        green.setStopIndex(5);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, purple);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, green);
    }

}
