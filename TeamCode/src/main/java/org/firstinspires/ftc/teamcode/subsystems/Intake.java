package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Intake implements Component {
    DcMotor intakeMotor, agitator;
    DistanceSensor frontColor, rightColor, leftColor;
    ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    static boolean isIntakeOn = false;
    double INTAKE_POWER = 0.9;
    double INTAKE_SHOOTING_POWER = 0.9;
    double INTAKE_FAST = 1.0;
    double REVERSAL_TIME = 500;
    double FIRE_POWER = 0.9;
    double AGITATOR_POWER = 0.2;
    double RAIL_UP = 0.5;
    double RAIL_DOWN = 1;
    double INTAKE_POWER_REVERSED = -0.9;
    double agitatorResetPosDone = 0.0;
    CRServo leftFireServo, rightFireServo, hood;
    Servo rail;
    ElapsedTime intakeRevTimer = new ElapsedTime();
    boolean intakeReversed = false;
    boolean agitatorResetRequest = false;
    boolean isShooting = false;


    @Override
    public void postInit() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "intake");
        agitator = ActiveOpMode.hardwareMap().get(DcMotor.class, "agitator"); //312 motor with 537.7 pulses per rev
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelLeft");
        rightFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelRight");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hood");
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        rail = ActiveOpMode.hardwareMap().get(Servo.class, "upperRail");
        frontColor = ActiveOpMode.hardwareMap().get(DistanceSensor.class, "frontColor");
        rightColor = ActiveOpMode.hardwareMap().get(DistanceSensor.class, "rightColor");
        leftColor = ActiveOpMode.hardwareMap().get(DistanceSensor.class, "leftColor");
        isIntakeOn = false;

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);


    }


    public void startRailDex() {
        isShooting = true;
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setTargetPosition(538);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

    }
    public void reverseIntake() {
        intakeMotor.setPower(-INTAKE_POWER);
        rail.setPosition(RAIL_UP);
    }
    public void stopReverseIntake() {
        intakeMotor.setPower(0);
    }
    public void resetRailDex() {
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
        agitator.setTargetPosition(0);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void startIntake() {
        if (agitator.isBusy()) {
            return;
        }
        intakeMotor.setPower(INTAKE_POWER);
        rail.setPosition(RAIL_UP);
        agitator.setTargetPosition(0);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void floatIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        rail.setPosition(RAIL_UP);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        agitator.setPower(0);
    }

    public void stopIntake() {
        intakeMotor.setPower(INTAKE_POWER_REVERSED);
        intakeRevTimer.reset();
        intakeReversed=true;
    }
    public void turnAgitator() {

        agitator.setTargetPosition(agitator.getCurrentPosition() + 538/4);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitatorResetRequest = true;
    }

    public void railDown(){
        rail.setPosition(RAIL_DOWN);
    }

    public Command stopIntake = new LambdaCommand()
            .setStart(() -> {
                isIntakeOn = false;
                intakeMotor.setPower(INTAKE_POWER_REVERSED);
                intakeRevTimer.reset();
                intakeReversed=true;
            })
            .setUpdate(() -> {
                if (intakeReversed && intakeRevTimer.milliseconds() >= REVERSAL_TIME) {
                    intakeMotor.setPower(0);
                    intakeReversed = false;
                    rail.setPosition(RAIL_DOWN);
                }
            })
            .setIsDone(() -> {
                return !intakeReversed;
            });

    public Command stopTransfer = new InstantCommand(
            () -> {
                leftFireServo.setPower(0);
                rightFireServo.setPower(0);
            }
    );
    /*public Command startIntake = new LambdaCommand()
            .setStart(() -> {
                intakeTimer.reset();
                startIntake();
                rail.setPosition(RAIL_UP);
            })
            .setUpdate(() -> {
            })
            .setStop(interrupted -> {
                stopIntake();
            })
            .setIsDone(() -> intakeTimer.seconds() > 1.0);*/
    public Command startIntake = new LambdaCommand()
            //() -> startIntake()
            .setStart(() -> {
                //shotTimer.reset();
                this.isIntakeOn = true;
                //intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
            //intakeMotor.setPower(INTAKE_POWER);
            rail.setPosition(RAIL_UP);
            agitator.setTargetPosition(0);
            agitator.setPower(AGITATOR_POWER);
            agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            })
            .setIsDone(() -> {
                return true; //shotTimer.seconds() > 2.0;
            })
            .setUpdate(() -> {
                //intakeMotor.setPower(INTAKE_POWER);
            })
            .setStop((interrupted) -> {
                //intakeMotor.setPower(INTAKE_POWER);
            });

    public Command slowIntake = new InstantCommand(
            () -> intakeMotor.setPower(INTAKE_SHOOTING_POWER)
    );
    public Command fastIntake = new InstantCommand(
            () -> intakeMotor.setPower(INTAKE_FAST)
    );
    public Command startTransfer = new InstantCommand(
            () -> {
                leftFireServo.setPower(FIRE_POWER);
                rightFireServo.setPower(FIRE_POWER);
                agitator.setPower(AGITATOR_POWER);
            }
    );
    public Command railDownAuto = new InstantCommand(
            () -> rail.setPosition(RAIL_DOWN)
    );
    public Command shootAllThree = new LambdaCommand()
            .setStart(() ->{
                shotTimer.reset();
                isShooting = true;
                railDown();
                startRailDex();
            })
            .setUpdate(() -> {
            })
            .setStop(interrupted -> {
                //flipper.setPosition(0.52);
                //resetRailDex();
            })
            .setIsDone(() -> (shotTimer.seconds() > 2.0)); //2.2 2

    public Command firewheelsOff = new InstantCommand(
            () -> {
                leftFireServo.setPower(0);
                rightFireServo.setPower(0);
            }
    );

    public Command resetRailDex = new InstantCommand(
            () -> resetRailDex()
    );


    public void update() {
        ActiveOpMode.telemetry().addData("left firewheel power", leftFireServo.getPower());
        ActiveOpMode.telemetry().addData("right firewheel power", rightFireServo.getPower());
        ActiveOpMode.telemetry().addData("agitator encoder", agitator.getCurrentPosition());
        ActiveOpMode.telemetry().addData("front color", frontColor.getDistance(DistanceUnit.INCH));
        ActiveOpMode.telemetry().addData("right color", rightColor.getDistance(DistanceUnit.INCH));
        ActiveOpMode.telemetry().addData("left color", frontColor.getDistance(DistanceUnit.INCH));

        if (isShooting && !agitator.isBusy()){
            isShooting = false;
            agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (isIntakeOn){
            intakeMotor.setPower(INTAKE_POWER);
        }

        if (intakeReversed && intakeRevTimer.milliseconds() >= REVERSAL_TIME) {
            intakeMotor.setPower(0);
            intakeReversed = false;
            rail.setPosition(RAIL_DOWN);
        }
        if (agitatorResetRequest && !agitator.isBusy()) {
            agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            agitator.setTargetPosition(0);
            agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            agitator.setPower(0);
            agitatorResetRequest = false;
        }
    }
}
