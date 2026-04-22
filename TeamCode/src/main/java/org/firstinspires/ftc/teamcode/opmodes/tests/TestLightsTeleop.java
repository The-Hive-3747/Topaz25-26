package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.TurretLights;

import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp
public class TestLightsTeleop extends NextFTCOpMode {
    TurretLights turretLights;
    String currentLightMode;
    Button g1a = new Button(() -> gamepad1.a);
    int lightMode = 0;
    @Override
    public void onInit() {
        turretLights = new TurretLights(hardwareMap, telemetry);
    }

    @Override
    public void onUpdate() {
        telemetry.addLine("Click gamepad 1 a to cycle lights");
        telemetry.addData("Current light mode", currentLightMode);
        telemetry.addData("light mode", lightMode);
        if (gamepad1.aWasPressed()) {
            lightMode++;
            if (lightMode == 1) {
                turretLights.redAlliance();
                currentLightMode = "red alliance";
            } else if (lightMode == 2) {
                turretLights.blueAlliance();
                currentLightMode = "blue alliance";
            } else if (lightMode == 3) {
               // turretLights.pPG();
                currentLightMode = "ppg";
            } else if (lightMode == 4) {
                turretLights.gPP();
                currentLightMode = "gpp";
            } else if (lightMode == 5) {
               // turretLights.pGP();
                currentLightMode = "pgp";
            } else if (lightMode == 6) {
                lightMode = 0;
            }
        }
        telemetry.update();
    }
}
