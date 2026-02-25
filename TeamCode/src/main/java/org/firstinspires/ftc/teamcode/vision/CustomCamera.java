package org.firstinspires.ftc.teamcode.vision;

import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import android.util.Size;
import org.openftc.easyopencv.OpenCvCamera;
import org.openftc.easyopencv.OpenCvCameraFactory;
import org.openftc.easyopencv.OpenCvCameraRotation;
import org.openftc.easyopencv.OpenCvWebcam;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@TeleOp
public class CustomCamera extends NextFTCOpMode {

    boolean hasBeenChanged = false;
    WatershedProc watershedProc;
    OpenCvWebcam camera;
    @Override
    public void onInit() {
        watershedProc = new WatershedProc();
        int cameraMonitorViewId = hardwareMap.appContext.getResources().getIdentifier("cameraMonitorViewId", "id", hardwareMap.appContext.getPackageName());
        WebcamName webcamName = hardwareMap.get(WebcamName.class, "Webcam 1");
        camera = OpenCvCameraFactory.getInstance().createWebcam(webcamName, cameraMonitorViewId);


        camera.openCameraDeviceAsync(new OpenCvCamera.AsyncCameraOpenListener() {
            @Override
            public void onOpened() {
                camera.startStreaming(320, 240, OpenCvCameraRotation.UPRIGHT, OpenCvWebcam.StreamFormat.MJPEG);
            }

            @Override
            public void onError(int errorCode) {

            }
        });

        telemetry.addLine("loaded!");
        telemetry.update();
    }

    @Override
    public void onUpdate() {
        camera.setPipeline(watershedProc);
        if (gamepad1.dpad_right && !hasBeenChanged) {
            watershedProc.changeViewMode();
            hasBeenChanged = true;
        } else if (!gamepad1.dpad_right && hasBeenChanged) {
            hasBeenChanged = false;
        }
        telemetry.addData("view mode", watershedProc.getViewMode());
        telemetry.addData("num obj found", watershedProc.getNumObjectsFound());
        telemetry.update();
        BindingManager.update();
    }
}