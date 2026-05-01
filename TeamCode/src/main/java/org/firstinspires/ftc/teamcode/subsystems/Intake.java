package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Artifact;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utilities.Motif;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class Intake implements Component {
    DcMotor intakeMotor;
    DcMotorEx agitator;
    TurretLights prismLights;
    GoBildaPrismDriver prism;
    NormalizedColorSensor frontColor, rightColor, leftColor;
    ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime colorTimer = new ElapsedTime();
    ElapsedTime agitator2Turn = new ElapsedTime();
    ElapsedTime autoShootInPartsFinishTime = new ElapsedTime();
    private double COLOR_CHECK_MS = 200;
    static boolean isIntakeOn = false;
    double INTAKE_POWER = 0.9;
    double INTAKE_SHOOTING_POWER = 0.9;
    double INTAKE_FAST = 1.0;
    double REVERSAL_TIME = 500;
    double RAIL_DOWN_TIME = 500;
    public static double FIRE_POWER = 1;//0.9
    public static double AGITATOR_POWER = 0.5;//0.6; //0.8;//0.2;//0.6;
    public static double RAIL_UP = 0.9; //0.72;// 0.22;//0.3//0.157;//0.3;//0.8;//0.5
    public static double RAIL_DOWN = 0.5;//1;//1;
    double INTAKE_POWER_REVERSED = -0.9;
    double agitatorResetPosDone = 0.0;
    static double AGITATOR_ENC_REVOLUTIONS_REV_V2 = 8192.0;
    static double  AGITATOR_ENC_REVOLUTIONS_GOBILDA_312 = 537.7;
    public static int AGITATOR_ENC = (int) AGITATOR_ENC_REVOLUTIONS_GOBILDA_312;
    int FIRST_ARTIFACT_POS = (AGITATOR_ENC/3);
    public static double SHOOT_IN_PARTS_FINISH_MS = 500;//1000;
    CRServo leftFireServo, rightFireServo, hood;
    Servo rail;
    ElapsedTime intakeRevTimer = new ElapsedTime();
    boolean intakeReversed = false;
    boolean agitatorResetRequest = false;
    boolean isRailRejecting = false;
    boolean isShooting = false;
    boolean isShootingInParts = false;
    boolean intakeStopping = false;
    boolean wasShotTimerReset = false;
    //boolean isRailDown = false;
    public static double SHOT_TIME_THRESHOLD_SEC = 2.25, SHOT_TIME_AFTER_AGITATOR_SEC = 0.6;

    Artifact leftArtifact = Artifact.EMPTY;
    Artifact rightArtifact = Artifact.EMPTY;
    Artifact frontArtifact = Artifact.EMPTY;
    private final double GREEN_THRESHOLD_CB_LT =  0.4994;
    private final double GREEN_MAYBE_THRESHOLD_CR_LT = 0.4999;
    private final double GREEN_THRESHOLD_CR_LT =  0.487;
    private final double PURPLE_THRESHOLD_CB_GT =  0.509;
    private final double PURPLE_THRESHOLD_CR_GT =  0.492;
    private final double ARTIFACT_THRESHOLD_CL_GT = 0.01;
    private boolean isAgitatorShootingInThirds = false, isAgitatorShootingInHalves = false, isRailDownRequested = false, useRailDownDetection = false;
    private ElapsedTime agitatorShootInPartsTimer = new ElapsedTime(), railDownTimer = new ElapsedTime();
    public double agitatorTurns = 0;
    public double agitatorAdjustNumber = 10;
    public double agitatorError = 0.3;
    public static double RAIL_DOWN_POS = 1.7;
    public static double RAIL_DOWN_POS_THRESHOLD = 0.2;
    public static double AGITATOR_SHOOT_IN_THIRDS_MS = 600;
    public static double AGITATOR_SHOOT_IN_HALVES_MS = 600; //600;
    YCbCr frontValues, rightValues, leftValues;
    double frontRed, frontGreen, frontBlue, rightRed, rightGreen, rightBlue, leftRed, leftGreen, leftBlue;
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    AnalogInput upperRail;


    @Override
    public void postInit() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "intake");
        agitator = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "agitator"); //312 motor with 537.7 pulses per rev
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelLeft");
        rightFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelRight");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hood");
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        //rightFireServo.setDirection(CRServo.Direction.REVERSE);
        rail = ActiveOpMode.hardwareMap().get(Servo.class, "upperRail");
        //rail.setDirection(Servo.Direction.REVERSE);
        frontColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "frontColor");
        rightColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "rightColor");
        leftColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "leftColor");
        prism = ActiveOpMode.hardwareMap().get(GoBildaPrismDriver.class, "lights");
        upperRail = ActiveOpMode.hardwareMap().get(AnalogInput.class, "upperRailEncoder");
        isIntakeOn = false;

        prismLights = new TurretLights(ActiveOpMode.hardwareMap(), ActiveOpMode.telemetry());

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//encoder has 8192 pulses per revolution (REV thru V2)

        /*PIDFCoefficients pid = agitator.getPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION);
        PIDFCoefficients pidNew = new PIDFCoefficients(pid.p*-1, pid.i *-1, pid.d * -1, pid.f * -1);
        agitator.setPIDFCoefficients(DcMotor.RunMode.RUN_TO_POSITION, pidNew);*/
    }

    public void useRailDownDetection() {
        useRailDownDetection = true;
    }

    public void resetAgitatorEncoder() {
        agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public void runFireWheels() {
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
    }

    public void startRailDex() {
        isShooting = true;
        stopIntake();
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setTargetPosition(AGITATOR_ENC);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void railDexInThirds() {
        stopIntake();
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setTargetPosition(agitator.getCurrentPosition() + AGITATOR_ENC/3);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitatorTurns +=1;
    }

    public void railDexInHalves() {
        stopIntake();
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setTargetPosition(agitator.getCurrentPosition() + AGITATOR_ENC/2);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitatorTurns +=1;
    }
    public void startRailDex2Turns() {
        agitator2Turn.reset();
        isShooting = true;
        stopIntake();
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setTargetPosition(AGITATOR_ENC/4);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        if (agitator2Turn.milliseconds() > 500) {
            agitator.setTargetPosition(3*AGITATOR_ENC/4);
            agitator.setPower(AGITATOR_POWER);
            agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        }
        agitatorTurns +=1;
    }

    public void startRailDexTime(){
        isShooting = true;
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        //agitator.setTargetPosition(AGITATOR_ENC);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void turnIsShootingTrue() {
        isShooting = true;
    }
    public void turnIsShootingFalse() {
        isShooting = false;
    }
    public void reverseIntake() {
        intakeMotor.setPower(-INTAKE_POWER);
        rail.setPosition(RAIL_UP);
        prismLights.intakeReversedLights();
    }
    public void stopReverseIntake() {
        intakeMotor.setPower(0);
    }
    public void resetRailDex() {
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
        agitator.setTargetPosition(0);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitator.setPower(AGITATOR_POWER);
    }
    public void startResetRailDex() {
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
        //agitator.setTargetPosition(0);
        agitator.setPower(0);
        //agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void startIntake() {
        isIntakeOn = true;
        intakeMotor.setPower(INTAKE_POWER);
        rail.setPosition(RAIL_UP);
        agitator.setTargetPosition(0);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitator.setPower(AGITATOR_POWER);
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
    }
    public void shiftIntake() {
        if (isIntakeOn) {
            rail.setPosition(RAIL_DOWN);
            agitator.setTargetPosition(FIRST_ARTIFACT_POS);
            agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            agitator.setPower(AGITATOR_POWER);
            //timer function?
            rail.setPosition(RAIL_UP);
        }
    }
    public void floatIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        rail.setPosition(RAIL_UP);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        agitator.setPower(0);
    }

    public void stopIntake() {
        isIntakeOn = false;
        //intakeMotor.setPower(INTAKE_POWER_REVERSED);
        railDown();
        intakeRevTimer.reset();
        intakeStopping = true;
    }
    public void turnAgitator() {

        agitator.setTargetPosition(agitator.getCurrentPosition() + AGITATOR_ENC/4);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitatorResetRequest = true;
    }

    public void railDown(){
        railDownTimer.reset();
        isRailDownRequested = true;
        rail.setPosition(RAIL_DOWN);
    }

    public void railUp(){
        rail.setPosition(RAIL_UP);
    }

    public Artifact detectArtifact(YCbCr values) {
        if (values.cL < ARTIFACT_THRESHOLD_CL_GT) {
            return Artifact.EMPTY;
        } else {
            if (values.cB > PURPLE_THRESHOLD_CB_GT || values.cR > PURPLE_THRESHOLD_CR_GT) {
                return Artifact.PURPLE;
            } else if (values.cB < GREEN_THRESHOLD_CB_LT || values.cR < GREEN_THRESHOLD_CR_LT) {
                return Artifact.GREEN;
            } else if (values.cR < GREEN_MAYBE_THRESHOLD_CR_LT) {
                return Artifact.MAYBE_GREEN;
            }
        }
        return Artifact.MAYBE_PURPLE;
    }

    class YCbCr {
        public double cL = 0;
        public double cB = 0;
        public double cR = 0;
        public YCbCr(double luminance, double cBlue, double cRed){
            cL = luminance;
            cB = cBlue;
            cR = cRed;
        }
        @Override
        public String toString() {
            return "cl:"+ String.format("%.4f",cL)+",cb:"+ String.format("%.4f",cB)+",cr:"+ String.format("%.4f", cR);
        }
    }
    public YCbCr colorConverter(double red, double green, double blue) {
        double cL = (0.299 * red) + (0.587 * green) + (0.114 * blue);
        double cB = 0.5 - (0.168736 * red) - (0.331264 * green) + (0.5 * blue);
        double cR = 0.5 + (0.5 * red) - (0.418688 * green) - (0.081312 * blue);
        return new YCbCr(cL, cB, cR);
    }

    public void getAllColorSensorValues() {
        frontRed = frontColor.getNormalizedColors().red;
        frontGreen = frontColor.getNormalizedColors().green;
        frontBlue = frontColor.getNormalizedColors().blue;
        rightRed = rightColor.getNormalizedColors().red;
        rightGreen = rightColor.getNormalizedColors().green;
        rightBlue = rightColor.getNormalizedColors().blue;
        leftRed = leftColor.getNormalizedColors().red;
        leftGreen = leftColor.getNormalizedColors().green;
        leftBlue = leftColor.getNormalizedColors().blue;

        frontValues = colorConverter(frontRed, frontGreen, frontBlue);
        rightValues = colorConverter(rightRed, rightGreen, rightBlue);
        leftValues = colorConverter(leftRed, leftGreen, leftBlue);
    }
    public boolean isFrontEmpty() {
        if (detectArtifact(frontValues) == Artifact.EMPTY) {
            return true;
        }else {
            return false;
        }
    }
    public void latchFrontColorSensor() {
        Artifact currentArtifact = detectArtifact(frontValues);
        if (leftArtifact == Artifact.EMPTY || rightArtifact == Artifact.EMPTY) {
            if (currentArtifact == Artifact.PURPLE) {
                frontArtifact = Artifact.MAYBE_PURPLE;
                return;
            } else if (currentArtifact == Artifact.GREEN) {
                frontArtifact = Artifact.MAYBE_GREEN;
                return;
            }
            frontArtifact = currentArtifact;
            return;
        }
        if (frontArtifact == Artifact.EMPTY || frontArtifact == Artifact.MAYBE_PURPLE || frontArtifact == Artifact.MAYBE_GREEN) {
            frontArtifact = currentArtifact;
            return;
        }
        if (frontArtifact == Artifact.PURPLE && currentArtifact == Artifact.GREEN) {
            frontArtifact = currentArtifact;
            return;
        }
    }

    public void latchRightColorSensor() {
        Artifact currentArtifact = detectArtifact(rightValues);
        if (rightArtifact == Artifact.EMPTY) {
            rightArtifact = currentArtifact;
            return;
        }
        if (rightArtifact == Artifact.PURPLE && currentArtifact == Artifact.GREEN) {
            rightArtifact = currentArtifact;
            return;
        }
    }

    public void latchLeftColorSensor() {
        Artifact currentArtifact = detectArtifact(leftValues);
        if (leftArtifact == Artifact.EMPTY) {
            leftArtifact = currentArtifact;
            return;
        }
        if (leftArtifact == Artifact.PURPLE && currentArtifact == Artifact.GREEN) {
            leftArtifact = currentArtifact;
            return;
        }
    }

    public void resetLatches() {
        frontArtifact = Artifact.EMPTY;
        rightArtifact = Artifact.EMPTY;
        leftArtifact = Artifact.EMPTY;

        // sets the luminance values to a point emulating no artifact
        // to ensure that we dont accidentally use old data after we shoot
        frontValues.cL = 0.001;
        rightValues.cL = 0.001;
        leftValues.cL = 0.001;
    }


    public Command stopIntake = new LambdaCommand()
            .setStart(() -> {
                isIntakeOn = false;
                intakeMotor.setPower(INTAKE_POWER_REVERSED);
                intakeRevTimer.reset();
                intakeReversed=true;
                railUp();
                //railDown();
            })
            .setUpdate(() -> {
                if (intakeReversed && intakeRevTimer.milliseconds() >= REVERSAL_TIME) {
                    intakeMotor.setPower(0);
                    intakeReversed = false;
                }
            })
            .setIsDone(() -> !intakeReversed);

    public InstantCommand stopIntakeNoReverse = new InstantCommand(
            () -> {
                isIntakeOn = false;
                railDown();
                intakeMotor.setPower(0);
            });

    public Command stopTransfer = new InstantCommand(
            () -> {
                leftFireServo.setPower(0);
                rightFireServo.setPower(0);
            }
    );
    public Command startIntake = new LambdaCommand()
            .setStart(() -> {
                isIntakeOn = true;
                rail.setPosition(RAIL_UP);
                agitator.setTargetPosition(0);
                agitator.setPower(AGITATOR_POWER);
                agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            })
            .setIsDone(() -> {
                return true;
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
            this::railDown
    );
    public Command shootAllThree() {
        ElapsedTime shotTimerAfterAgitator = new ElapsedTime();
        return new LambdaCommand()
                .setStart(() ->{
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                    shotTimer.reset();
                    isShooting = true;
                    railDown();
                    startRailDex();
                })
                .setUpdate(() -> {
                    if (!agitator.isBusy() && !wasShotTimerReset) {
                        shotTimerAfterAgitator.reset();
                        wasShotTimerReset = true;
                    }
                })
                .setStop(interrupted -> {})
                //in order to reset agitator fully it needs 2.75 seconds but its usually covered in the driving.
                .setIsDone(() -> (shotTimer.seconds() > SHOT_TIME_THRESHOLD_SEC ));//|| // Checks if agitator is not busy, then gives a 0.25s buffer
        // (shotTimerAfterAgitator.seconds() > SHOT_TIME_AFTER_AGITATOR_SEC && wasShotTimerReset)));
    }

    public Command shootAllThreeInHalves() {
        ElapsedTime shotTimerAfterAgitator = new ElapsedTime();
        return new LambdaCommand()
                .setStart(() ->{
                    leftFireServo.setPower(FIRE_POWER);
                    rightFireServo.setPower(FIRE_POWER);
                    shotTimer.reset();
                    isShooting = true;
                    isShootingInParts = true;
                    railDown();
                    shootInHalves();
                })
                .setUpdate(() -> {
                    /*if (!agitator.isBusy() && !wasShotTimerReset && agitatorTurns) {
                        shotTimerAfterAgitator.reset();
                        wasShotTimerReset = true;
                    }*/
                })
                .setStop(interrupted -> {isShootingInParts = false;})
                .setIsDone(() -> (autoShootInPartsFinishTime.milliseconds() > SHOOT_IN_PARTS_FINISH_MS && !isShootingInParts));
    }

    public Command firewheelsOff = new InstantCommand(
            () -> {
                isShooting = false;
                leftFireServo.setPower(0);
                rightFireServo.setPower(0);
            }
    );

    public Command resetRailDex = new InstantCommand(
            () -> resetRailDex()
    );

    public Command railUpAuto = new InstantCommand(
            () -> rail.setPosition(RAIL_UP)
    );

    public Command reverseIntake = new InstantCommand(
            () -> intakeReversed = true
    );

    public Motif getMotif() {
        return new Motif(leftArtifact, frontArtifact, rightArtifact);
    }


    public void update() {
        if(colorTimer.milliseconds() > COLOR_CHECK_MS) {
            getAllColorSensorValues();
            latchFrontColorSensor();
            latchRightColorSensor();
            latchLeftColorSensor();
            colorTimer.reset();
        }

        if (isAgitatorShootingInThirds) {
            if (agitatorTurns >= 3) {
                agitator.setTargetPosition(AGITATOR_ENC);
                agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                agitator.setPower(AGITATOR_POWER);
                isAgitatorShootingInThirds = false;
                agitatorTurns = 0;

                // Turning isShooting to true here because the agitator becomes not busy on every third, and we dont want to reset
                isShooting = true;
            } else {
                if (agitatorTurns == 0) {
                    railDexInThirds();
                    agitatorShootInPartsTimer.reset();
                }
                if (agitatorShootInPartsTimer.milliseconds() > AGITATOR_SHOOT_IN_THIRDS_MS) {
                    railDexInThirds();
                    agitatorShootInPartsTimer.reset();
                }
            }
        }

        if (isAgitatorShootingInHalves) {
            if (agitatorTurns >= 2) {
                agitator.setTargetPosition(AGITATOR_ENC);
                agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                agitator.setPower(AGITATOR_POWER);
                isAgitatorShootingInHalves = false;
                agitatorTurns = 0;
                isShootingInParts = false;
                autoShootInPartsFinishTime.reset();


                // Turning isShooting to true here because the agitator becomes not busy on every third, and we dont want to reset
                isShooting = true;
            } else {
                if (agitatorTurns == 0) {
                    railDexInHalves();
                    agitatorShootInPartsTimer.reset();
                }
                if (agitatorShootInPartsTimer.milliseconds() > AGITATOR_SHOOT_IN_HALVES_MS) {
                    railDexInHalves();
                    agitatorShootInPartsTimer.reset();
                }
            }
        }

        if (isShooting && !agitator.isBusy() &&!isAgitatorShootingInHalves && !isAgitatorShootingInThirds){
            isShooting = false;
            agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (isShooting || isAgitatorShootingInThirds || isAgitatorShootingInHalves){
            leftFireServo.setPower(FIRE_POWER);
            rightFireServo.setPower(FIRE_POWER);
        }

        if (isIntakeOn){
            resetLatches();
            railUp();
            intakeMotor.setPower(INTAKE_POWER);
            prismLights.railUpLights();
            prismLights.intakeOnLights();
        }

        if (intakeReversed && intakeRevTimer.milliseconds() >= REVERSAL_TIME) {
            intakeMotor.setPower(0);
            intakeReversed = false;
            railDown();
            prismLights.railDownLights();
        }
        if(intakeStopping && intakeRevTimer.milliseconds() >= RAIL_DOWN_TIME){
            intakeStopping = false;
            intakeReversed = true;
            intakeMotor.setPower(INTAKE_POWER_REVERSED);
            intakeRevTimer.reset();
            prismLights.intakeReversedLights();
        }
        if (agitatorResetRequest && !agitator.isBusy()) {
            agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            agitator.setTargetPosition(0);
            agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            agitator.setPower(0);
            //agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
            agitatorResetRequest = false;
        }

        if (isRailDownRequested && useRailDownDetection) {
            if (railDownTimer.milliseconds() > RAIL_DOWN_TIME) {
                if (upperRail.getVoltage() >= RAIL_DOWN_POS + RAIL_DOWN_POS_THRESHOLD && !isRailRejecting) {
                    railUp();
                    intakeRevTimer.reset();
                    agitator.setTargetPosition(agitator.getCurrentPosition() + 5*AGITATOR_ENC/360);
                    agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    agitator.setPower(AGITATOR_POWER);

                    // This calls the intake reverse for a time logic
                    intakeStopping = true;
                    isRailRejecting = true;
                    railDownTimer.reset();
                } else if (isRailRejecting ) {
                    agitator.setTargetPosition(0);
                    agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                    agitator.setPower(AGITATOR_POWER);
                    isRailRejecting = false;
                }else {
                    isRailDownRequested = false;
                }
            }
        }
        /*if(intakeStopping && upperRail.getVoltage() >= RAIL_DOWN_POS){
            railUp();
            intakeReversed = true;
            intakeRevTimer.reset();
        }*/

        /*if (agitatorTurns >= agitatorAdjustNumber) { //This is code to adjust for our error in converting doubles to integers
            agitator.setTargetPosition((int)(agitator.getCurrentPosition() - agitatorError*agitatorAdjustNumber));
        }*/

        /*if(rightArtifact == Artifact.GREEN){
            prismLights.gPP();
        }
        if(frontArtifact == Artifact.PURPLE){
            prismLights.pPG();
        }
        if(frontArtifact == Artifact.GREEN){
            prismLights.pGP();
        }*/

    }

    public void shootInThirds() {
        isAgitatorShootingInThirds = true;
    }
    public void shootInHalves() {
        isAgitatorShootingInHalves = true;
    }

    public String getIntakeState() {
        if (isIntakeOn) {
            return "On";
        }
        if (intakeReversed) {
            return "Reversed";
        }
        return "Off";
    }

    public String getFirewheelState() {
        if (isShooting) {
            return "On";
        }
        return "Off";
    }

    public int getAgitatorPos(){
        return agitator.getCurrentPosition();
    }

    public void telemetry() {
        ActiveOpMode.telemetry().addLine("---- Intake ----");
        ActiveOpMode.telemetry().addData("Left Firewheel Power", leftFireServo.getPower());
        ActiveOpMode.telemetry().addData("Right Firewheel Power", rightFireServo.getPower());
        ActiveOpMode.telemetry().addData("Agitator Position", agitator.getCurrentPosition());
        ActiveOpMode.telemetry().addData("Agitator Is Busy", agitator.isBusy());
        ActiveOpMode.telemetry().addData("Artifact Colors", this.getMotif());
        ActiveOpMode.telemetry().addData("Intake Power", intakeMotor.getPower());
        ActiveOpMode.telemetry().addData("Rail Position", upperRail.getVoltage());
    }
}
