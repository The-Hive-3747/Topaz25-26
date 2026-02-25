package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;

@Disabled
@TeleOp
public class intakeTest extends LinearOpMode {
    private DcMotor intakeMotor;
    private CRServo leftFireServo;
    private CRServo rightFireServo;
    private CRServo sideWheelServo;
    private Servo flipper;
    private DcMotor flywheelLeft;
    private DcMotor flywheelRight;

    private double INTAKE_POWER = 0.9;//0.5;
    private double INTAKE_POWER_STEP = 0.05;
    private double FIRE_POWER = 0.9;//0.3;
    private double FIRE_POWER_STEP = 0.05;
    private double FLYWHEEL_POWER = 0.8;//0.6;
    private double FLYWHEEL_POWER_STEP = 0.05;
    private double HOOD_POSITION = 0.0;
    private boolean isIntakeOn = false;
    private boolean wasAPressed = false;
    private boolean wasX2Pressed = false;
    private boolean wasB2Pressed = false;
    private boolean wasB1Pressed = false;
    private boolean isFireServoOn = false;
    private boolean wasA2Pressed = false;
    private boolean wasY2Pressed = false;
    private boolean wasY1Pressed = false;
    private boolean isFlywheelOn = false;
    private boolean was2DpadLeftPressed = false;
    private boolean was2DpadRightPressed = false;
    private boolean wasXPressed = false;
    private boolean isFlipped = false;
    private boolean was1DpadLeftPressed = false;
    private boolean was1DpadRightPressed = false;


    @Override
    public void runOpMode() {
        intakeMotor = hardwareMap.get(DcMotor.class, "transfer");
        leftFireServo = hardwareMap.get(CRServo.class, "left_firewheel");
        rightFireServo = hardwareMap.get(CRServo.class, "right_firewheel");
        flywheelLeft = hardwareMap.get(DcMotor.class, "flywheelLeft");
        flywheelRight = hardwareMap.get(DcMotor.class, "flywheelRight");

        sideWheelServo = hardwareMap.get(CRServo.class, "side-wheel");
        flipper = hardwareMap.get(Servo.class, "flipper");
        Servo hoodServo = hardwareMap.get(Servo.class, "hoodServo");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        //leftFireServo.setDirection(CRServo.Direction.REVERSE);
        rightFireServo.setDirection(CRServo.Direction.REVERSE);
        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        waitForStart();

        flipper.setPosition(0.4);
        //hoodServo.setPosition(HOOD_POSITION);
        while (opModeIsActive()){
            //this is for intake
            if(gamepad1.a && !wasAPressed){
                if (!isIntakeOn){
                    intakeMotor.setPower(INTAKE_POWER);
                    isIntakeOn = true;
                }else{
                    intakeMotor.setPower(0);
                    isIntakeOn = false;
                }
                wasAPressed = true;
            }
            if(!gamepad1.a && wasAPressed){
                wasAPressed = false;
            }
            if(gamepad1.x && !wasXPressed){
                if (!isFlipped){
                    flipper.setPosition(0.27); //0.18
                    isFlipped = true;
                }else{
                    flipper.setPosition(0.4);
                    isFlipped = false;
                }
                wasXPressed = true;
            }
            if(!gamepad1.x && wasXPressed){
                wasXPressed = false;
            }
            if(gamepad2.x && !wasX2Pressed){
                wasX2Pressed = true;
                INTAKE_POWER = INTAKE_POWER  - INTAKE_POWER_STEP;
                if(INTAKE_POWER < -1.0){
                    INTAKE_POWER = -1.0;
                }
                if(isIntakeOn){
                    intakeMotor.setPower(INTAKE_POWER);
                }
            }
            if(!gamepad2.x && wasX2Pressed){
                wasX2Pressed = false;
            }
            if(gamepad2.b && !wasB2Pressed){
                wasB2Pressed = true;
                INTAKE_POWER = INTAKE_POWER  + INTAKE_POWER_STEP;
                if(INTAKE_POWER > 1.0){
                    INTAKE_POWER = 1.0;
                }
                if(isIntakeOn){
                    intakeMotor.setPower(INTAKE_POWER);
                }
            }
            if(!gamepad2.b && wasB2Pressed){
                wasB2Pressed = false;
            }
            //this is for fireservos
            if(gamepad1.b && !wasB1Pressed){
                if (!isFireServoOn){
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                    sideWheelServo.setPower(FIRE_POWER);
                    isFireServoOn = true;
                }else{
                    leftFireServo.setPower(0);
                    rightFireServo.setPower(0);
                    sideWheelServo.setPower(0);
                    isFireServoOn = false;
                }
                wasB1Pressed = true;
            }
            if(!gamepad1.b && wasB1Pressed){
                wasB1Pressed = false;
            }
            if(gamepad2.a && !wasA2Pressed){
                wasA2Pressed = true;
                FIRE_POWER = FIRE_POWER - FIRE_POWER_STEP;
                if(FIRE_POWER < -1.0){
                    FIRE_POWER = -1.0;
                }
                if(isFireServoOn){
                    sideWheelServo.setPower(FIRE_POWER);
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                }
            }
            if(!gamepad2.a && wasA2Pressed){
                wasA2Pressed = false;
            }
            if(gamepad2.y && !wasY2Pressed){
                wasY2Pressed = true;
                FIRE_POWER = FIRE_POWER  + FIRE_POWER_STEP;
                if(FIRE_POWER > 1.0){
                    FIRE_POWER = 1.0;
                }
                if(isFireServoOn){
                    sideWheelServo.setPower(FIRE_POWER);
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                }
            }
            if(!gamepad2.y && wasY2Pressed){
                wasY2Pressed = false;
            }
            //this is for the flywheel
            if(gamepad1.y && !wasY1Pressed){
                if (!isFlywheelOn){
                    flywheelLeft.setPower(FLYWHEEL_POWER);
                    flywheelRight.setPower(FLYWHEEL_POWER);
                    isFlywheelOn = true;
                }else{
                    flywheelLeft.setPower(0);
                    flywheelRight.setPower(0);
                    isFlywheelOn = false;
                }
                wasY1Pressed = true;
            }
            if(!gamepad1.y && wasY1Pressed){
                wasY1Pressed = false;
            }
            if (gamepad1.dpad_left && !was1DpadLeftPressed){
                was1DpadLeftPressed = true;
                if (hoodServo.getPosition() <= 0.1) {
                    HOOD_POSITION = 0;
                    hoodServo.setPosition(HOOD_POSITION);
                } else {
                    HOOD_POSITION -= 0.1;
                    hoodServo.setPosition(HOOD_POSITION);
                }
            }
            if (!gamepad1.dpad_left && was1DpadLeftPressed) {
                was1DpadLeftPressed = false;
            }
            if (gamepad1.dpad_right && !was1DpadRightPressed){
                was1DpadRightPressed = true;
                if (hoodServo.getPosition() >= 0.9) {
                    HOOD_POSITION = 1;
                    hoodServo.setPosition(HOOD_POSITION);
                } else {
                    HOOD_POSITION += 0.1;
                    hoodServo.setPosition(HOOD_POSITION);
                }
            }
            if (!gamepad1.dpad_right && was1DpadRightPressed) {
                was1DpadRightPressed = false;
            }
            if(gamepad2.dpad_left && !was2DpadLeftPressed){
                was2DpadLeftPressed = true;
                FLYWHEEL_POWER = FLYWHEEL_POWER - FLYWHEEL_POWER_STEP;
                if(FLYWHEEL_POWER < -1.0){
                    FLYWHEEL_POWER_STEP = -1.0;
                }
                if(isFlywheelOn){
                    flywheelLeft.setPower(FLYWHEEL_POWER);
                    flywheelRight.setPower(FLYWHEEL_POWER);

                }
            }
            if(!gamepad2.dpad_left && was2DpadLeftPressed){
                was2DpadLeftPressed = false;
            }
            if(gamepad2.dpad_right && !was2DpadRightPressed){
                was2DpadRightPressed = true;
                FLYWHEEL_POWER = FLYWHEEL_POWER  + FLYWHEEL_POWER_STEP;
                if(FLYWHEEL_POWER > 1.0){
                    FLYWHEEL_POWER = 1.0;
                }
                if(isFlywheelOn){
                    flywheelLeft.setPower(FLYWHEEL_POWER);
                    flywheelRight.setPower(FLYWHEEL_POWER);

                }
            }
            if(!gamepad2.dpad_right && was2DpadRightPressed){
                was2DpadRightPressed = false;
            }
            telemetry.addData("Intake Power", INTAKE_POWER);
            telemetry.addData("Fire Servos Power", FIRE_POWER);
            telemetry.addData("Flywheel Power", FLYWHEEL_POWER);
            //telemetry.addData("Flywheel Velocity", flywheelRight.getVelocity());
            telemetry.addData("Hood Servo Position", hoodServo.getPosition());
            telemetry.addData("Press Gamepad 2 X to decrease the intake power by 5%", "");
            telemetry.addData("Press Gamepad 2 B to increase the intake power by 5%", "");
            telemetry.addData("Press Gamepad 1 A to toggle the intake","");
            telemetry.addData("Press Gamepad 2 A to decrease the fire servos power by 5%", "");
            telemetry.addData("Press Gamepad 2 Y to increase the fire servos power by 5%", "");
            telemetry.addData("Press Gamepad 1 B to toggle the fire servos","");
            telemetry.addData("Press Gamepad 2 Dpad Left to decrease the flywheel power by 5%", "");
            telemetry.addData("Press Gamepad 2 Dpad Right increase the flywheel power by 5%", "");
            telemetry.addData("Press Gamepad 1hood Y to toggle the flywheel","");
            telemetry.addData("Press Gamepad 1 X to toggle the Flipper","");
            telemetry.addData("Press Gamepad 1 Dpad Left to decrease the hood servo position", "");
            telemetry.addData("Press Gamepad 1 Dpad Right increase the hood servo position", "");
            telemetry.update();
        }
    }
}
