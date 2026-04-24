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
    PrismAnimations.Solid noShoot = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid purple1 = new PrismAnimations.Solid(Color.MAGENTA);
    PrismAnimations.Solid purple2 = new PrismAnimations.Solid(Color.MAGENTA);
    PrismAnimations.Solid intaking = new PrismAnimations.Solid(Color.PINK);
    PrismAnimations.Solid endgameWarning = new PrismAnimations.Solid(Color.YELLOW);
    PrismAnimations.Solid endgameFinalWarning = new PrismAnimations.Solid(Color.RED);
    PrismAnimations.Solid manualMode = new PrismAnimations.Solid(Color.BLUE);

    HardwareMap hardwareMap;
    Telemetry telemetry;
    int brightness = 30;
    int startIndexBottom = 0;
    int stopIndexBottom = 11;
    int startIndexBall1Right = 12;
    int stopIndexBall1Right = 13;
    int startIndexBall2Right = 14;
    int stopIndexBall2Right = 15;
    int startIndexBall3Right = 16;
    int stopIndexBall3Right = 17;
    int startIndexBall3Left = 18;
    int stopIndexBall3Left = 19;
    int startIndexBall2Left = 20;
    int stopIndexBall2Left = 21;
    int startIndexBall1Left = 22;
    int stopIndexBall1Left = 23;
    int startIndexTopRight = 12;
    int stopIndexTopRight = 17;
    int startIndexTopLeft = 18;
    int stopIndexTopLeft = 23;


    public TurretLights(HardwareMap hm, Telemetry tm) {
        hardwareMap = hm;
        telemetry = tm;
        prism = hardwareMap.get(GoBildaPrismDriver.class, "lights");

        prism.setStripLength(24);

        green1.setBrightness(brightness);
        green2.setBrightness(brightness);
        purple1.setBrightness(brightness);
        purple2.setBrightness(brightness);
    }

    public void readyToShoot() {
        shootNow.setStartIndex(startIndexBottom);
        shootNow.setStopIndex(stopIndexBottom);
        shootNow.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, shootNow);
    }

    public void endgameWarning() {
        endgameWarning.setStartIndex(startIndexBottom);
        endgameWarning.setStopIndex(stopIndexTopLeft);
        endgameWarning.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_9, endgameWarning);
    }

    public void manualMode() {
        manualMode.setStartIndex(startIndexBottom);
        manualMode.setStopIndex(stopIndexTopLeft);
        manualMode.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_9, manualMode);
    }

    public void clearEndgameWarning() {
        prism.clearAllAnimations();
    }

    public void endgameFinalWarning() {
        endgameFinalWarning.setStartIndex(startIndexBottom);
        endgameFinalWarning.setStopIndex(stopIndexTopLeft);
        endgameFinalWarning.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_9, endgameFinalWarning);
    }

    public void intaking() {
        intaking.setStartIndex(startIndexBottom);
        intaking.setStopIndex(stopIndexBottom);
        intaking.setBrightness(brightness);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_1, intaking);
    }

    public void notReadyToShoot() {
        noShoot.setBrightness(brightness);
        noShoot.setStartIndex(startIndexBottom);
        noShoot.setStopIndex(stopIndexBottom);
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
        green1.setStartIndex(startIndexBall1Right);
        green1.setStopIndex(stopIndexBall1Right);
        purple1.setStartIndex(startIndexTopRight);
        purple1.setStopIndex(stopIndexTopRight);
        green2.setStartIndex(startIndexBall1Left);
        green2.setStopIndex(stopIndexBall1Left);
        purple2.setStartIndex(startIndexTopLeft);
        purple2.setStopIndex(stopIndexTopLeft);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, purple1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, purple2);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, green1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_6, green2);
    }

    public void pGP() {
        green1.setStartIndex(startIndexBall2Right);
        green1.setStopIndex(stopIndexBall2Right);
        purple1.setStartIndex(startIndexTopRight);
        purple1.setStopIndex(stopIndexTopRight);
        green2.setStartIndex(startIndexBall2Left);
        green2.setStopIndex(stopIndexBall2Left);
        purple2.setStartIndex(startIndexTopLeft);
        purple2.setStopIndex(stopIndexTopLeft);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, purple1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, purple2);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, green1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_6, green2);
    }

    public void pPG() {
        green1.setStartIndex(startIndexBall3Right);
        green1.setStopIndex(stopIndexBall3Right);
        purple1.setStartIndex(startIndexTopRight);
        purple1.setStopIndex(stopIndexTopRight);
        green2.setStartIndex(startIndexBall3Left);
        green2.setStopIndex(stopIndexBall3Left);
        purple2.setStartIndex(startIndexTopLeft);
        purple2.setStopIndex(stopIndexTopLeft);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_3, purple1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_4, purple2);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_5, green1);
        prism.insertAndUpdateAnimation(GoBildaPrismDriver.LayerHeight.LAYER_6, green2);
    }


}
