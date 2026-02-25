package org.firstinspires.ftc.teamcode.opmodes;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.ActiveOpMode;

@Autonomous
public class ExampleAuto extends OpMode {
    DcMotor frontLeft, frontRight, backLeft, backRight;
    // if wanted add shoot motors here
    ElapsedTime autoStageTimer = new ElapsedTime();
    int autoStage = 0;

    @Override
    public void init(){
        //use your own motor names defined in your code
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeftMotor");
        frontRight = hardwareMap.get(DcMotor.class, "frontRightMotor");
        backLeft = hardwareMap.get(DcMotor.class, "backLeftMotor");
        backRight = hardwareMap.get(DcMotor.class, "backRightMotor");
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        autoStage = 0;
    }

    @Override
    public void init_loop(){
        telemetry.addData("Auto Initialized", "Waiting for Start");
    }

    @Override
    public void loop(){
        switch(autoStage){
            case 0:
                driveForward();
                nextStage();
                break;
            case 1:
                if(autoStageTimer.seconds() >= 1.5){
                    stopDriving();
                    nextStage();
                }
                break;
            default:
                stopDriving();
                telemetry.addData("Auto Done", "");
                break;

        }
        telemetry.addData("Stage Number", autoStage);
        telemetry.update();
    }

    public void nextStage(){
        autoStage ++;
        autoStageTimer.reset();
    }
    public void stopDriving(){
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
    public void driveForward(){
        frontLeft.setPower(0.4);
        frontRight.setPower(0.4);
        backLeft.setPower(0.4);
        backRight.setPower(0.4);
    }
    public void driveBackwards(){
        frontLeft.setPower(-0.4);
        frontRight.setPower(-0.4);
        backLeft.setPower(-0.4);
        backRight.setPower(-0.4);
    }
    public void strafeLeft(){
        frontLeft.setPower(-0.4);
        frontRight.setPower(0.4);
        backLeft.setPower(0.4);
        backRight.setPower(-0.4);
    }
    public void strafeRight(){
        frontLeft.setPower(0.4);
        frontRight.setPower(-0.4);
        backLeft.setPower(-0.4);
        backRight.setPower(0.4);
    }
    public void turnLeft(){
        frontLeft.setPower(-0.4);
        frontRight.setPower(0.4);
        backLeft.setPower(-0.4);
        backRight.setPower(0.4);
    }
    public void turnRight(){
        frontLeft.setPower(0.4);
        frontRight.setPower(-0.4);
        backLeft.setPower(0.4);
        backRight.setPower(-0.4);
    }
    public void shoot(){
        //add your shooting code here, if you want to shoot make sure you declare it with your other motors
    }
}
