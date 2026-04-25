package org.firstinspires.ftc.teamcode.opmodes.tests;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.NormalizedRGBA;
import com.qualcomm.robotcore.hardware.SwitchableLight;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
@TeleOp(name = "encoder Tester")
//@Disabled
public class EncoderTester extends NextFTCOpMode {
    CRServo turretLeft, turretRight;
private TouchSensor limitSwitch;
    DcMotorEx flywheelLeft, flywheelRight, intakeMotor, agitator;
    AnalogInput upperRail;
    @Override
    public void onInit() {
        flywheelLeft = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelLeft");
        limitSwitch = ActiveOpMode.hardwareMap().get(TouchSensor.class, "limitSwitch");
        //flywheelLeft.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelRight = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelRight");
        //flywheelRight.setDirection(DcMotorEx.Direction.FORWARD);
        //turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "intake");
        turretLeft = ActiveOpMode.hardwareMap().get(CRServo.class, "turretLeft");
        turretRight = ActiveOpMode.hardwareMap().get(CRServo.class, "turretRight");

        upperRail = ActiveOpMode.hardwareMap().get(AnalogInput.class, "upperRailEncoder");

        agitator = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "agitator"); //312 motor with 537.7 pulses per rev
//        agitator.setDirection(DcMotorSimple.Direction.REVERSE);

        agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        //turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        //turret.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

    }
    @Override
    public void onUpdate() {
        telemetry.addData("flywheelLeft", flywheelLeft.getCurrentPosition());
        telemetry.addData("flywheelRight (hood enc)", flywheelRight.getCurrentPosition());
        telemetry.addData("flywheelLeft p", flywheelLeft.getPower());
        telemetry.addData("flywheelRight p", flywheelRight.getPower());
        telemetry.addData("flywheel left vel", flywheelLeft.getVelocity());
        telemetry.addData("flywheel external vel W CONVERSION",  ((double) flywheelRight.getVelocity()*60)/8192); // 8192 is CPR
        telemetry.addData("flywheelRight vel", flywheelRight.getVelocity());
        telemetry.addData("intakeMotor (turret enc)", intakeMotor.getCurrentPosition());
        telemetry.addData("limit switch", limitSwitch.getValue());
        telemetry.addData("agitator", agitator.getCurrentPosition());

        if (gamepad1.a) {
            flywheelLeft.setPower(1);
        } else {
            flywheelLeft.setPower(0);
        }
        if (gamepad1.b) {
            flywheelRight.setPower(1);
        } else {
            flywheelRight.setPower(0);
        }
        //telemetry.addData("turret", turret.getCurrentPosition());

        telemetry.addData("port fly right", ActiveOpMode.hardwareMap().get("flyWheelRight").getConnectionInfo());
        telemetry.addData("port fly left", ActiveOpMode.hardwareMap().get("flyWheelLeft").getConnectionInfo());

        telemetry.addData("Motor turret", "Turret encoder");
        telemetry.addData("Motor transfer", "Hood encoder");

        telemetry.addData("Upper Rail", upperRail.getVoltage());
        telemetry.addData("Upper Rail", upperRail.getMaxVoltage());

        telemetry.update();
    }


}
