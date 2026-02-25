package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
@TeleOp(name = "encoder Tester")
public class EncoderTester extends NextFTCOpMode {

    DcMotorEx flywheelBottom, flywheelTop, turret, intakeMotor;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;
    NormalizedRGBA colors;
    @Override
    public void onInit() {
        flywheelBottom = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelBottom");
        //flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelTop = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelTop");
        //flywheelRight.setDirection(DcMotorEx.Direction.FORWARD);
        turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "transfer");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "intakeColor");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"intakeColor");


        flywheelTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelBottom.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if(colorSensor instanceof SwitchableLight){
            ((SwitchableLight)colorSensor).enableLight(true);
        }

    }
    @Override
    public void onUpdate() {
        colors = colorSensor.getNormalizedColors();
        telemetry.addData("Distance (inch)",distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Red",colors.red);
        telemetry.addData("Green",colors.green);
        telemetry.addData("Blue",colors.blue);
        telemetry.addData("flywheelBottom", flywheelBottom.getCurrentPosition());
        telemetry.addData("flywheelTop", flywheelTop.getCurrentPosition());
        telemetry.addData("flywheel external vel", flywheelBottom.getVelocity());
        telemetry.addData("flywheel external vel W CONVERSION",  ((double) flywheelTop.getVelocity()*60)/8192); // 8192 is CPR
        telemetry.addData("flywheelTop vel", flywheelTop.getVelocity());
        telemetry.addData("intakeMotor", intakeMotor.getCurrentPosition());
        telemetry.addData("turret", turret.getCurrentPosition());

        telemetry.addData("port fly bottom", ActiveOpMode.hardwareMap().get("flywheelBottom").getConnectionInfo());
        telemetry.addData("port fly top", ActiveOpMode.hardwareMap().get("flywheelTop").getConnectionInfo());
        telemetry.addData("port intake", ActiveOpMode.hardwareMap().get("transfer").getConnectionInfo());

        telemetry.addData("Motor turret", "Turret encoder");
        telemetry.addData("Motor flywheel top", "Empty");
        telemetry.addData("Motor flywheel bottom", "Flywheel bottom encoder");
        telemetry.addData("Motor transfer", "Hood encoder");

        telemetry.update();
    }


}
