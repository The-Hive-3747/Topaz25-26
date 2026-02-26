package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
@TeleOp(name = "encoder Tester")
public class EncoderTester extends NextFTCOpMode {

    DcMotorEx flywheelLeft, flywheelRight, intakeMotor;
    NormalizedColorSensor colorSensor;
    DistanceSensor distanceSensor;
    NormalizedRGBA colors;
    @Override
    public void onInit() {
        flywheelLeft = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelLeft");
        //flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelRight");
        //flywheelRight.setDirection(DcMotorEx.Direction.FORWARD);
        //turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake");
        colorSensor = hardwareMap.get(NormalizedColorSensor.class, "frontColor");
        distanceSensor = hardwareMap.get(DistanceSensor.class,"frontColor");


        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        if(colorSensor instanceof SwitchableLight){
            ((SwitchableLight)colorSensor).enableLight(true);
        }

    }
    @Override
    public void onUpdate() {
        colors = colorSensor.getNormalizedColors();
        telemetry.addData("Front Distance (inch)",distanceSensor.getDistance(DistanceUnit.INCH));
        telemetry.addData("Front Red",colors.red);
        telemetry.addData("Front Green",colors.green);
        telemetry.addData("Front Blue",colors.blue);
        telemetry.addData("flywheelLeft", flywheelLeft.getCurrentPosition());
        telemetry.addData("flywheelRight", flywheelRight.getCurrentPosition());
        telemetry.addData("flywheel external vel", flywheelLeft.getVelocity());
        telemetry.addData("flywheel external vel W CONVERSION",  ((double) flywheelRight.getVelocity()*60)/8192); // 8192 is CPR
        telemetry.addData("flywheelTop vel", flywheelRight.getVelocity());
        telemetry.addData("intakeMotor", intakeMotor.getCurrentPosition());
        //telemetry.addData("turret", turret.getCurrentPosition());

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
