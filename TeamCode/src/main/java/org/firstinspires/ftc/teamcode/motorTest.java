package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;

import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.ftc.NextFTCOpMode;
import dev.nextftc.hardware.impl.MotorEx;


//@Disabled
@TeleOp(name= "motor test")
public class motorTest extends NextFTCOpMode {
    {
        addComponents(
                //drive = new FieldCentricDrive(),
                BindingsComponent.INSTANCE
        );
    }
    FieldCentricDrive drive;
    MotorEx frontLeft, frontRight, backLeft, backRight, flyLeft, flyRight;
    DcMotorEx turret;
    CRServo leftFireServo, rightFireServo, sideWheelServo, hood;



    @Override
    public void onInit() {
        turret = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "turret");

        turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        turret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        telemetry.addData("turret ticks", turret.getCurrentPosition());
        telemetry.update();
        frontLeft = new MotorEx("frontLeftMotor").brakeMode();
        frontRight = new MotorEx("frontRightMotor").brakeMode();
        backLeft = new MotorEx("backLeftMotor").brakeMode();
        backRight = new MotorEx("backRightMotor").brakeMode();
        flyLeft = new MotorEx("flywheelLeft");
        flyRight = new MotorEx("flywheelRight");
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "left_firewheel");
        sideWheelServo = ActiveOpMode.hardwareMap().get(CRServo.class, "side-wheel");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        rightFireServo.setDirection(CRServo.Direction.REVERSE);
    }

    @Override
    public void onUpdate() {
        telemetry.addData("turret ticks", (turret.getCurrentPosition()*90)/6100);
        telemetry.update();
        if (gamepad1.left_trigger > 0.1) {
            leftFireServo.setPower(0.9);
        } else {
            leftFireServo.setPower(0);
        }
        if (gamepad1.right_trigger > 0.1) {
            sideWheelServo.setPower(0.9);
        } else {
            sideWheelServo.setPower(0);
        }
        if (gamepad1.dpad_up) {
            turret.setPower(0.5); } else {
            turret.setPower(0);
        }
        if (gamepad1.dpad_down) {
            flyRight.setPower(1); } else {
            flyRight.setPower(0);
        }
        if (gamepad1.a) {
            frontLeft.setPower(1);
        } else {
            frontLeft.setPower(0);
        }
        if (gamepad1.b) {
            frontRight.setPower(1);
        } else {
            frontRight.setPower(0);
        }
        if (gamepad1.x) {
            backLeft.setPower(1);
        } else {
            backLeft.setPower(0);
        }
        if (gamepad1.y) {
            backRight.setPower(1);
        } else {
            backRight.setPower(0);
        }
    }
}
