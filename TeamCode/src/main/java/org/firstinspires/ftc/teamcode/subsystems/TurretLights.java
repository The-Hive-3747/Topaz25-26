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
    PrismAnimations.Solid green1 = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid green2 = new PrismAnimations.Solid(Color.GREEN);
    PrismAnimations.Solid noShoot = new PrismAnimations.Solid(Color.MAGENTA);
    PrismAnimations.Solid purple1 = new PrismAnimations.Solid(Color.MAGENTA);
    PrismAnimations.Solid purple2 = new PrismAnimations.Solid(Color.MAGENTA);
    PrismAnimations.Solid purple3 = new PrismAnimations.Solid(Color.MAGENTA);

    HardwareMap hardwareMap;
    Telemetry telemetry;
    int brightness = 30;
    int startIndexBottom = 0;
    int stopIndexBottom = 11;
    int startIndexTop1 = 12;
    int stopIndexTop1 = 17;
    int startIndexTop2 = 18;
    int stopIndexTop2 = 23;


    public TurretLights(HardwareMap hm, Telemetry tm) {
        hardwareMap = hm;
        telemetry = tm;
        prism = hardwareMap.get(GoBildaPrismDriver.class, "lights");

        prism.setStripLength(24);

        green1.setBrightness(brightness);
        green2.setBrightness(brightness);
        purple1.setBrightness(brightness);
        purple2.setBrightness(brightness);
        purple3.setBrightness(brightness);
    }

    public void readyToShoot() {
        shootNow.setStartIndex(startIndexBottom);
        shootNow.setStopIndex(stopIndexBottom);
        shootNow.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, shootNow);
    }

    public void notReadyToShoot() {
        noShoot.setStartIndex(startIndexBottom);
        noShoot.setStartIndex(stopIndexBottom);
        noShoot.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, noShoot);
    }

    public void redAlliance() {
        red.setBrightness(brightness);
        red.setStartIndex(startIndexBottom);
        red.setStopIndex(stopIndexBottom);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, red);
    }

    public void blueAlliance() {
        blue.setBrightness(brightness);
        blue.setStartIndex(startIndexBottom);
        blue.setStopIndex(stopIndexBottom);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_0, blue);
    }

    public void gPP() {
        green1.setStartIndex(startIndexTop1);
        green1.setStopIndex(startIndexTop1+1);
        purple1.setStartIndex(startIndexTop1);
        purple1.setStopIndex(stopIndexTop1);
        green2.setStartIndex(stopIndexTop2-1);
        green2.setStopIndex(stopIndexTop2);
        purple2.setStartIndex(startIndexTop2);
        purple2.setStopIndex(stopIndexTop2);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, purple1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, purple2);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, green1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_6, green2);
    }


}
