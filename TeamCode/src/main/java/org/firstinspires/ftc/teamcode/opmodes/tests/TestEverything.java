package org.firstinspires.ftc.teamcode.opmodes.tests;

import static org.firstinspires.ftc.teamcode.subsystems.Intake.RAIL_DOWN;
import static org.firstinspires.ftc.teamcode.subsystems.Intake.RAIL_UP;
import static dev.nextftc.bindings.Bindings.button;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name = "test everything")
public class TestEverything extends NextFTCOpMode {
    enum TestName{
        CONFIG,
        DRIVE_WHEELS,
        PINPOINT,
        INTAKE,
        RAIL,
        AGITATOR,
        FIREWHEELS,
        FLYWHEEL_LEFT,
        FLYWHEEL_RIGHT,
        FLYWHEEL_VEL,
        HOOD,
        TURRET,
        TURRET_ZERO,
        LIMELIGHT,
        LIGHTS,
        COLOR_SENSORS,
        RESULTS
    }
    ElapsedTime testTimer = new ElapsedTime();
    private int whichWheel = 0;
    private boolean goToNextTest = false;
    private boolean telemetrySent = false;
    private boolean pinpointHeadingPass = false;
    private boolean pinpointXPass = false;
    private boolean pinpointYPass = false;
    private double WHEEL_POWER = 0.5;
    private double TEST_TIMER_THRESHOLD = 1000;
    private static double FLYWHEEL_VEL_TEST = 3000, FLYWHEEL_VEL_THRESHOLD = 100, FLYWHEEL_TIME_THRESHOLD_MS = 2000, FLYWHEEL_VEL = 0, FLYWHEEL_TEST_TIME = 2000;
    private static double HOOD_TEST_POS = 6000,  HOOD_POS_THRESHOLD = 100, HOOD_TIME_THRESHOLD_MS = 500, HOOD_TEST_TIME = 1500, HOOD_POS = 0;
    private static double TURRET_TEST_POS = 45, TURRET_POS_THRESHOLD = 1, TURRET_TIME_THRESHOLD_MS = 500, TURRET_TEST_TIME = 3000, TURRET_POS = 0;
    private static boolean isTestingHood = false, hasHoodPassed = false, hasHoodTestCompleted = false;
    private static boolean isTestingFlywheel = false, hasFlywheelPassed = false, hasFlywheelTestCompleted = false;
    private static boolean isTestingTurret = false, hasTurretPassed = false, hasTurretTestCompleted = false;
    double pinpointHeading = 0;
    double pinpointX = 0;
    double pinpointY = 0;
    Flywheel flywheel;
    TestName currentTest = TestName.CONFIG;
    DcMotorEx leftFront, rightFront, rightBack, leftBack, agitator, intake, flywheelLeft, flywheelRight;
    Servo rail;
    CRServo hoodServo, firewheelLeft, firewheelRight, turretLeft, turretRight;
    Hood hood;
    Turret turret;
    GoBildaPinpointDriver pinpoint;
    NormalizedColorSensor frontColor, leftColor, rightColor;
    GoBildaPrismDriver prism;
    Button g1A;

    @Override
    public void onInit(){
        currentTest = TestName.CONFIG;
        telemetrySent = false;
        pinpointHeadingPass = false;
        pinpointXPass = false;
        pinpointYPass = false;
        telemetry.addLine("PUT THE ROBOT ON THE SPARKLE OR ELSE BAD THINGS HAPPEN");
        telemetry.addLine("------------------------------------------------------------");
        telemetry.addLine("Once the start button is pressed it is going to test each");
        telemetry.addLine("subsystem individually. Press A to go through the tests.");
        telemetry.addLine("Some tests the robot doesn't know if it's working or not,");
        telemetry.addLine("so you'll have to confirm that it's working during the test");
        telemetry.addLine("Press X to confirm that it's working, or press y if it doesn't work");
        telemetry.update();

    }

    @Override
    public void onWaitForStart(){

    }

    @Override
    public void onStartButtonPressed(){
        g1A = button(() -> gamepad1.a);
        g1A.whenBecomesTrue(()-> {
            goToNextTest = true;
        });
    }

    @Override
    public void onUpdate(){
        BindingManager.update();
        telemetry.clearAll();
        telemetry.addData("Current Test", currentTest);
        runTests();
    }
    public void testConfig(){
        boolean somethingFailed = false;
        if(!telemetrySent) {
            leftFront = hardwareMap.tryGet(DcMotorEx.class, "frontLeftMotor");
            rightFront = hardwareMap.tryGet(DcMotorEx.class, "frontRightMotor");
            leftBack = hardwareMap.tryGet(DcMotorEx.class, "backLeftMotor");
            rightBack = hardwareMap.tryGet(DcMotorEx.class, "backRightMotor");
            agitator = hardwareMap.tryGet(DcMotorEx.class, "agitator");
            intake = hardwareMap.tryGet(DcMotorEx.class, "intake");
            flywheelLeft = hardwareMap.tryGet(DcMotorEx.class, "flyWheelLeft");
            flywheelRight = hardwareMap.tryGet(DcMotorEx.class, "flyWheelRight");
            rail = hardwareMap.tryGet(Servo.class, "upperRail");
            hoodServo = hardwareMap.tryGet(CRServo.class, "hood");
            firewheelLeft = hardwareMap.tryGet(CRServo.class, "fireWheelLeft");
            firewheelRight = hardwareMap.tryGet(CRServo.class, "fireWheelRight");
            turretLeft = hardwareMap.tryGet(CRServo.class, "turretLeft");
            turretRight = hardwareMap.tryGet(CRServo.class, "turretRight");
            pinpoint = hardwareMap.tryGet(GoBildaPinpointDriver.class, "odo");
            frontColor = hardwareMap.tryGet(NormalizedColorSensor.class, "frontColor");
            rightColor = hardwareMap.tryGet(NormalizedColorSensor.class, "rightColor");
            leftColor = hardwareMap.tryGet(NormalizedColorSensor.class, "leftColor");
            prism = hardwareMap.tryGet(GoBildaPrismDriver.class, "lights");
        }
            telemetry.addLine("Press A to move to next test");
            telemetry.addLine("This tests to see if each named part is in the robot config");
            if (null == leftFront) {
                telemetry.addData("front left motor ", "FAIL");
                somethingFailed = true;
            }
            if (null == leftBack) {
                telemetry.addData("back left motor", "FAIL");
                somethingFailed = true;
            }
            if (null == rightFront) {
                telemetry.addData("front right motor", "FAIL");
                somethingFailed = true;
            }
            if (null == rightBack) {
                telemetry.addData("back right motor", "FAIL");
                somethingFailed = true;
            }
            if (null == agitator) {
                telemetry.addData("agitator", "FAIL");
                somethingFailed = true;
            }
            if (null == intake) {
                telemetry.addData("intake", "FAIL");
                somethingFailed = true;
            }
            if (null == flywheelLeft) {
                telemetry.addData("flywheel left", "FAIL");
                somethingFailed = true;
            }
            if (null == flywheelRight) {
                telemetry.addData("flywheel right", "FAIL");
                somethingFailed = true;
            }
            if (null == rail) {
                telemetry.addData("rail", "FAIL");
                somethingFailed = true;
            }
            if (null == hoodServo) {
                telemetry.addData("hood", "FAIL");
                somethingFailed = true;
            }
            if (null == firewheelLeft) {
                telemetry.addData("firewheel left", "FAIL");
                somethingFailed = true;
            }
            if (null == firewheelRight) {
                telemetry.addData("firewheel right", "FAIL");
                somethingFailed = true;
            }
            if (null == turretLeft) {
                telemetry.addData("turret left", "FAIL");
                somethingFailed = true;
            }
            if (null == turretRight) {
                telemetry.addData("turret right", "FAIL");
                somethingFailed = true;
            }
            if (null == pinpoint) {
                telemetry.addData("pinpoint", "FAIL");
                somethingFailed = true;
            }
            if (null == frontColor) {
                telemetry.addData("front color", "FAIL");
                somethingFailed = true;
            }
            if (null == leftColor) {
                telemetry.addData("left color", "FAIL");
                somethingFailed = true;
            }
            if (null == rightColor) {
                telemetry.addData("right color", "FAIL");
                somethingFailed = true;
            }
            if (null == prism) {
                telemetry.addData("prism", "FAIL");
                somethingFailed = true;
            }

            if (!somethingFailed) {
                telemetry.addLine("ALL PASS");
            }

            if (null != leftFront) {
                telemetry.addData("front left motor ", "PASS");
                leftFront.setDirection(DcMotorEx.Direction.REVERSE);
            }
            if (null != leftBack) {
                telemetry.addData("back left motor", "PASS");
                leftBack.setDirection(DcMotorEx.Direction.REVERSE);
            }
            if (null != rightFront) {
                telemetry.addData("front right motor", "PASS");
            }
            if (null != rightBack) {
                telemetry.addData("back right motor", "PASS");
            }
            if (null != agitator) {
                telemetry.addData("agitator", "PASS");
            }
            if (null != intake) {
                telemetry.addData("intake", "PASS");
            }
            if (null != flywheelLeft) {
                telemetry.addData("flywheel left", "PASS");
            }
            if (null != flywheelRight) {
                telemetry.addData("flywheel right", "PASS");
            }
            if (null != rail) {
                telemetry.addData("rail", "PASS");
            }
            if (null != hoodServo) {
                telemetry.addData("hood", "PASS");
            }
            if (null != firewheelLeft) {
                telemetry.addData("firewheel left", "PASS");
            }
            if (null != firewheelRight) {
                telemetry.addData("firewheel right", "PASS");
            }
            if (null != turretLeft) {
                telemetry.addData("turret left", "PASS");
            }
            if (null != turretRight) {
                telemetry.addData("turret right", "PASS");
            }
            if (null != pinpoint) {
                telemetry.addData("pinpoint", "PASS");
            }
            if (null != frontColor) {
                telemetry.addData("front color", "PASS");
            }
            if (null != leftColor) {
                telemetry.addData("left color", "PASS");
            }
            if (null != rightColor) {
                telemetry.addData("right color", "PASS");
            }
            if (null != prism) {
                telemetry.addData("prism", "PASS");
            }
            telemetry.update();
            telemetrySent = true;
    }
    public void testDriveWheels(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test alternates each wheel going forward");
        if(testTimer.milliseconds() > TEST_TIMER_THRESHOLD){
            whichWheel ++;
            whichWheel = whichWheel % 4;
            setAllWheelsToOff();
            testTimer.reset();
        }
        switch(whichWheel){
            case 0:
                if(leftFront != null){
                    leftFront.setPower(WHEEL_POWER);
                    telemetry.addLine("Left Front wheel is moving forward");
                }else{
                    telemetry.addLine("Skipping left front wheel because it doesn't exist in config");
                }
                break;
            case 1:
                if(leftBack != null){
                    leftBack.setPower(WHEEL_POWER);
                    telemetry.addLine("Left Back wheel is moving forward");
                }else{
                    telemetry.addLine("Skipping left back wheel because it doesn't exist in config");
                }
                break;
            case 2:
                if(rightFront != null){
                    rightFront.setPower(WHEEL_POWER);
                    telemetry.addLine("right front wheel is moving forward");
                }else{
                    telemetry.addLine("Skipping right front wheel because it doesn't exist in config");
                }
                break;
            case 3:
                if(rightBack != null){
                    rightBack.setPower(WHEEL_POWER);
                    telemetry.addLine("right back wheel is moving forward");
                }else{
                    telemetry.addLine("Skipping right back wheel because it doesn't exist in config");
                }
                break;
        }
        telemetry.update();
    }

    private void setAllWheelsToOff() {
        if(leftFront != null){
            leftFront.setPower(0);
        }
        if(leftBack != null){
            leftBack.setPower(0);
        }
        if(rightFront != null){
            rightFront.setPower(0);
        }
        if(rightBack != null) {
            rightBack.setPower(0);
        }
    }

    public void testPinpoint(){
        pinpoint.update();
        telemetry.addLine("Press A to go to the next test");
        telemetry.addLine("This is to see if the pinpoint works");
        telemetry.addLine("Turn the robot to the right to test heading");
        telemetry.addLine("Spin the forward/back encoder from front to back");
        telemetry.addLine("Spin the lateral encoder from left to right");
        if(null != pinpoint) {
            if (pinpointHeading - pinpoint.getHeading(AngleUnit.DEGREES) > 10) {
                pinpointHeadingPass = true;
            }
            if (pinpoint.getPosX(DistanceUnit.INCH) - pinpointX > 5) {
                pinpointXPass = true;
            }
            if (pinpoint.getPosY(DistanceUnit.INCH) - pinpointY > 5) {
                pinpointYPass = true;
            }
            telemetry.addData("Pinpoint Status", pinpoint.getDeviceStatus());
            if (pinpointHeadingPass) {
                telemetry.addData("Pinpoint heading", "PASS");
            } else {
                telemetry.addData("Pinpoint heading", "FAIL");
            }
            if (pinpointXPass) {
                telemetry.addData("Pinpoint x deadwheel", "PASS");
            } else {
                telemetry.addData("Pinpoint x deadwheel", "FAIL");
            }
            if (pinpointYPass) {
                telemetry.addData("Pinpoint y deadwheel", "PASS");
            } else {
                telemetry.addData("Pinpoint y deadwheel", "FAIL");
            }
            telemetry.addData("Pinpoint heading", pinpoint.getHeading(AngleUnit.DEGREES));
            telemetry.addData("Pinpoint X", pinpoint.getPosX(DistanceUnit.INCH));
            telemetry.addData("Pinpoint Y", pinpoint.getPosY(DistanceUnit.INCH));
        }else{
            telemetry.addLine("FAIL: Pinpoint skipped because not found in config");
        }
        telemetry.update();
    }
    public void testIntake() {
        if (testTimer.milliseconds() > TEST_TIMER_THRESHOLD) {
            telemetry.addLine("Press A to move to the next test");
            telemetry.addLine("This test alternates turning on and off the intake");
            if (intake != null) {
                if (intake.getPower() > 0) {
                    intake.setPower(0);
                    telemetry.addLine("intake is stopped");
                } else {
                    intake.setPower(WHEEL_POWER);
                    telemetry.addLine("intake is moving forward");
                }
            } else {
                telemetry.addLine("Skipping intake because it doesn't exist in config");
            }
            testTimer.reset();
            telemetry.update();
        }
    }
    public void testRail(){
        if (testTimer.milliseconds() > TEST_TIMER_THRESHOLD) {
            telemetry.addLine("Press A to move to the next test");
            telemetry.addLine("This test alternates moving the rail up and down");
            if (rail != null) {
                if (rail.getPosition() > RAIL_DOWN) {
                    rail.setPosition(RAIL_DOWN);
                    telemetry.addLine("rail is down");
                } else {
                    rail.setPosition(RAIL_UP);
                    telemetry.addLine("rail is up");
                }
            } else {
                telemetry.addLine("Skipping rail because it doesn't exist in config");
            }
            testTimer.reset();
            telemetry.update();
        }
    }
    public void testAgitator(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test alternates moving the agitator 180 deg");
        if (!agitator.isBusy()) {
            if (agitator != null) {
                if (agitator.getTargetPosition() > 0) {
                    agitator.setTargetPosition(0);
                    agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    agitator.setPower(Intake.AGITATOR_POWER);
                    telemetry.addLine("agitator is at 0");
                } else {
                    agitator.setTargetPosition(Intake.AGITATOR_ENC / 2);
                    agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    agitator.setPower(Intake.AGITATOR_POWER);
                    telemetry.addLine("agitator is 180 deg");
                }
            } else {
                telemetry.addLine("Skipping agitator because it doesn't exist in config");
            }
            testTimer.reset();
            telemetry.update();
        }
    }
    public void testFirewheels(){
        if (testTimer.milliseconds() > TEST_TIMER_THRESHOLD) {
            telemetry.addLine("Press A to move to the next test");
            telemetry.addLine("This test alternates turning on and off the firewheels");
            if (firewheelRight  != null && firewheelLeft != null) {
                if (firewheelRight.getPower() > 0) {
                    firewheelRight.setPower(0);
                    firewheelLeft.setPower(0);
                    telemetry.addLine("firewheels are stopped");
                } else {
                    firewheelRight.setPower(Intake.FIRE_POWER);
                    firewheelLeft.setPower(Intake.FIRE_POWER);
                    telemetry.addLine("firewheels are on");
                }
            } else {
                telemetry.addLine("Skipping firewheels because one or both don't exist in config");
            }
            testTimer.reset();
            telemetry.update();
        }
    }
    public void testFlywheelLeft(){
        if (testTimer.milliseconds() > TEST_TIMER_THRESHOLD) {
            telemetry.addLine("Press A to move to the next test");
            telemetry.addLine("This test alternates turning on and off the flywheel left");
            if (flywheelLeft != null) {
                if (flywheelLeft.getPower() > 0) {
                    flywheelLeft.setPower(0);
                    telemetry.addLine("flywheel left is stopped");
                } else {
                    flywheelLeft.setPower(WHEEL_POWER);
                    telemetry.addLine("flywheel left is moving forward");
                }
            } else {
                telemetry.addLine("Skipping flywheel left because it doesn't exist in config");
            }
            testTimer.reset();
            telemetry.update();
        }
    }
    public void testFlywheelRight(){
        if (testTimer.milliseconds() > TEST_TIMER_THRESHOLD) {
            telemetry.addLine("Press A to move to the next test");
            telemetry.addLine("This test alternates turning on and off the flywheel right");
            if (flywheelRight != null) {
                if (flywheelRight.getPower() > 0) {
                    flywheelRight.setPower(0);
                    telemetry.addLine("flywheel right is stopped");
                } else {
                    flywheelRight.setPower(WHEEL_POWER);
                    telemetry.addLine("flywheel right is moving forward");
                }
            } else {
                telemetry.addLine("Skipping flywheel right because it doesn't exist in config");
            }
            testTimer.reset();
            telemetry.update();
        }
    }
    public void testFlywheelVel(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test sets the Flywheel to a speed and tests if it can hold that speed");
        if (flywheel != null ) {
            if (!isTestingFlywheel) {
                flywheel.setTargetVel(FLYWHEEL_VEL_TEST);
                isTestingFlywheel = true;
                testTimer.reset();
            } else if (testTimer.milliseconds() < FLYWHEEL_TIME_THRESHOLD_MS) {
                telemetry.addLine("Spinning up flywheel");
            } else if (!hasFlywheelTestCompleted) {
                FLYWHEEL_VEL = flywheel.getVel();
                if (Math.abs(FLYWHEEL_VEL - FLYWHEEL_VEL_TEST) <= FLYWHEEL_VEL_THRESHOLD) {
                    telemetry.addLine("Flywheel PASS");
                    hasFlywheelPassed = true;
                } else {
                    telemetry.addLine("Flywheel FAILED");
                    hasFlywheelPassed = false;
                }
                hasFlywheelTestCompleted = true;
            } else {
                if (hasFlywheelPassed) {
                    telemetry.addLine("Flywheel PASS");
                } else {
                    telemetry.addLine("Flywheel FAILED");
                }
                telemetry.addData("Flywheel Vel Tested", FLYWHEEL_VEL);
                flywheel.setTargetVel(0);
            }
            telemetry.addData("Flywheel Current Vel", flywheel.getVel());
            telemetry.addData("Flywheel Goal", FLYWHEEL_VEL_TEST);
            flywheel.update();
        } else {
            telemetry.addLine("Skipping flywheel because at least one flywheel does not exist in config");
        }
        telemetry.update();
    }
    public void testHood(){
        if (hood != null){
            telemetry.addLine("Press A to move to the next test");
            telemetry.addLine("This test moves the hood to a position");
            if (!isTestingHood) {
                hood.setGoal(HOOD_TEST_POS);
                isTestingHood = true;
                testTimer.reset();
            } else if (testTimer.milliseconds() < HOOD_TEST_TIME) {
                telemetry.addLine("Moving the hood up");
                if (testTimer.milliseconds() < HOOD_TIME_THRESHOLD_MS && hood.getHoodPosition() == 0) {
                    telemetry.addLine("Stopping Hood because hood position is 0");
                    hood.setHoodPower(0);
                    hasHoodPassed = false;
                    hasHoodTestCompleted = true;
                }
            } else if (!hasHoodTestCompleted) {
                HOOD_POS = hood.getHoodPosition();
                if (Math.abs(HOOD_POS - HOOD_TEST_POS) <= HOOD_POS_THRESHOLD) {
                    telemetry.addLine("Hood PASS");
                    hasHoodPassed = true;
                } else {
                    telemetry.addLine("Flywheel FAILED");
                    hasHoodPassed = false;
                }
                hasHoodTestCompleted = true;
            } else {
                if (hasHoodPassed) {
                    telemetry.addLine("Hood PASS");
                } else {
                    telemetry.addLine("Hood FAILED");
                }
                telemetry.addData("Hood Pos Tested", HOOD_POS);
                hood.setGoal(0);
                hood.setHoodPower(0);
            }
            telemetry.addData("Hood Current POs", hood.getHoodPosition());
            telemetry.addData("Hood Goal", HOOD_TEST_POS);
            hood.update();
        } else {
            telemetry.addLine("Skipping hood because either the servo doesnt exist, or the FlywheelRight Encoder");
        }
        telemetry.update();
    }
    public void testTurret(){
        if (turret != null){
            telemetry.addLine("Press A to move to the next test");
            telemetry.addLine("This test moves the turret to a position");
            if (!isTestingTurret) {
                turret.setFixedAngleCustom(TURRET_TEST_POS);
                isTestingTurret = true;
                testTimer.reset();
            } else if (testTimer.milliseconds() < TURRET_TEST_TIME) {
                telemetry.addLine("Moving the turret");
                if (testTimer.milliseconds() < TURRET_TIME_THRESHOLD_MS && turret.getTurretAngle() == 0) {
                    telemetry.addLine("Stopping Turret because hood position is 0");
                    turret.setTurretStateOff();
                    turret.setTurretPower(0);
                    hasTurretPassed = false;
                    hasTurretTestCompleted = true;
                }
            } else if (!hasTurretTestCompleted) {
                TURRET_POS = turret.getTurretAngle();
                if (Math.abs(TURRET_POS - TURRET_TEST_POS) <= TURRET_POS_THRESHOLD) {
                    telemetry.addLine("Turret PASS");
                    hasTurretPassed = true;
                } else {
                    telemetry.addLine("Turret FAILED");
                    hasTurretPassed = false;
                }
                hasTurretTestCompleted = true;
            } else {
                if (hasTurretPassed) {
                    telemetry.addLine("Turret PASS");
                } else {
                    telemetry.addLine("Turret FAILED");
                }
                telemetry.addData("Turret Pos Tested", TURRET_POS);
                turret.setTurretPower(0);
                turret.setTurretStateOff();
            }
            telemetry.addData("Turret Current Pos", turret.getTurretAngle());
            telemetry.addData("Turret Goal", TURRET_TEST_POS);
            turret.update();
        } else {
            telemetry.addLine("Skipping turret because either the servos don't exist, or the Intake Encoder");
        }
        telemetry.update();
    }
    public void testTurretZero(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test has not been completed yet");
    }
    public void testLimelight(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test has not been completed yet");
    }
    public void testLights(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test has not been completed yet");
    }
    public void testColorSensors(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test has not been completed yet");
    }
    public void showResults(){
        telemetry.addLine("Press A to move to the next test");
        telemetry.addLine("This test has not been completed yet");
    }
    public void runTests(){
        switch(currentTest){
            case CONFIG:
                testConfig();
                if(goToNextTest){
                    testTimer.reset();
                    currentTest = TestName.DRIVE_WHEELS;
                    goToNextTest = false;
                }
                break;
            case DRIVE_WHEELS:
                testDriveWheels();
                if(goToNextTest){
                    currentTest = TestName.PINPOINT;
                    goToNextTest = false;
                    pinpoint.update();
                    setAllWheelsToOff();
                    if (pinpoint != null) {
                        pinpointHeading = pinpoint.getHeading(AngleUnit.DEGREES);
                        pinpointX = pinpoint.getPosX(DistanceUnit.INCH);
                        pinpointY = pinpoint.getPosY(DistanceUnit.INCH);
                    }
                }
                break;
            case PINPOINT:
                testPinpoint();
                if(goToNextTest){
                    currentTest = TestName.INTAKE;
                    goToNextTest = false;
                }
                break;
            case INTAKE:
                testIntake();
                if(goToNextTest){
                    if(intake != null){
                        intake.setPower(0);
                    }
                    currentTest = TestName.RAIL;
                    goToNextTest = false;
                }
                break;
            case RAIL:
                testRail();
                if(goToNextTest){
                    if(agitator != null){
                        agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    }
                    currentTest = TestName.AGITATOR;
                    goToNextTest = false;
                }
                break;
            case AGITATOR:
                testAgitator();
                if(goToNextTest){
                    if(firewheelRight != null && firewheelLeft != null){
                        firewheelRight.setDirection(DcMotorSimple.Direction.REVERSE);
                        firewheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                    }
                    currentTest = TestName.FIREWHEELS;
                    goToNextTest = false;
                }
                break;
            case FIREWHEELS:
                testFirewheels();
                if(goToNextTest){
                    if(flywheelLeft != null){
                        flywheelLeft.setDirection(DcMotorSimple.Direction.REVERSE);
                    }
                    if (firewheelLeft != null) {
                        firewheelLeft.setPower(0);
                    }
                    if (firewheelRight != null) {
                        firewheelRight.setPower(0);
                    }
                    currentTest = TestName.FLYWHEEL_LEFT;
                    goToNextTest = false;
                }
                break;
            case FLYWHEEL_LEFT:
                testFlywheelLeft();
                if(goToNextTest){
                    if(flywheelLeft != null){
                        flywheelLeft.setPower(0);
                    }
                    currentTest = TestName.FLYWHEEL_RIGHT;
                    goToNextTest = false;
                }
                break;
            case FLYWHEEL_RIGHT:
                testFlywheelRight();
                if(goToNextTest){
                    if(flywheelRight != null){
                        flywheelRight.setPower(0);
                    }
                    if(flywheelLeft != null && flywheelRight != null){
                        flywheel = new Flywheel();
                        flywheel.postInit();
                    }
                    currentTest = TestName.FLYWHEEL_VEL;
                    goToNextTest = false;
                }
                break;
            case FLYWHEEL_VEL:
                testFlywheelVel();
                if(goToNextTest) {
                    if (flywheelRight != null) {
                        flywheelRight.setPower(0);
                    }
                    if (flywheelLeft != null) {
                        flywheelLeft.setPower(0);
                    }
                    if (flywheel != null) {
                        flywheel.setTargetVel(0);
                    }
                    if (flywheelRight != null && hoodServo != null) {
                        hood = new Hood(flywheelRight);
                        hood.init();
                    }
                    currentTest = TestName.HOOD;
                    goToNextTest = false;
                }
                break;
            case HOOD:
                testHood();
                if(goToNextTest){
                    if (hood != null) {
                        hood.setGoal(0);
                        hood.setHoodPower(0);
                    }
                    if (turretLeft != null && turretRight != null && intake != null) {
                        turret = new Turret();
                        turret.preInit();
                        turret.postInit();
                        turret.setTurretStateOff();
                    }
                    currentTest = TestName.TURRET;
                    goToNextTest = false;
                }
                break;
            case TURRET:
                testTurret();
                if(goToNextTest){
                    currentTest = TestName.TURRET_ZERO;
                    goToNextTest = false;
                }
                break;
            case TURRET_ZERO:
                testTurretZero();
                if(goToNextTest){
                    currentTest = TestName.LIMELIGHT;
                    goToNextTest = false;
                }
                break;
            case LIMELIGHT:
                testLimelight();
                if(goToNextTest){
                    currentTest = TestName.LIGHTS;
                    goToNextTest = false;
                }
                break;
            case LIGHTS:
                testLights();
                if(goToNextTest){
                    currentTest = TestName.COLOR_SENSORS;
                    goToNextTest = false;
                }
                break;
            case COLOR_SENSORS:
                testColorSensors();
                if(goToNextTest){
                    currentTest = TestName.RESULTS;
                    goToNextTest = false;
                }
                break;
            case RESULTS:
                showResults();
                break;

        }
    }
}
