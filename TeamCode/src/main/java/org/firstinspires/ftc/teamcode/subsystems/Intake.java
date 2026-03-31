package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.utilities.Artifact;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Intake implements Component {
    DcMotor intakeMotor;
    DcMotorEx agitator;
    NormalizedColorSensor frontColor, rightColor, leftColor;
    ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    static boolean isIntakeOn = false;
    double INTAKE_POWER = 0.9;
    double INTAKE_SHOOTING_POWER = 0.9;
    double INTAKE_FAST = 1.0;
    double REVERSAL_TIME = 500;
    double FIRE_POWER = 1;
    double AGITATOR_POWER = 0.6;
    double RAIL_UP = 0.5;
    double RAIL_DOWN = 1;
    double INTAKE_POWER_REVERSED = -0.9;
    double agitatorResetPosDone = 0.0;
    double AGITATOR_ENC_REVOLUTIONS_GOBILDA_312 = 537.7;
    int AGITATOR_ENC = (int) AGITATOR_ENC_REVOLUTIONS_GOBILDA_312;
    CRServo leftFireServo, rightFireServo, hood;
    Servo rail;
    ElapsedTime intakeRevTimer = new ElapsedTime();
    boolean intakeReversed = false;
    boolean agitatorResetRequest = false;
    boolean isShooting = false;

    Artifact leftArtifact = Artifact.EMPTY;
    Artifact rightArtifact = Artifact.EMPTY;
    Artifact frontArtifact = Artifact.EMPTY;
    private final double GREEN_THRESHOLD_CB_LT = 0.4994;
    private final double GREEN_MAYBE_THRESHOLD_CR_LT = 0.4999;
    private final double GREEN_THRESHOLD_CR_LT = 0.487;
    private final double PURPLE_THRESHOLD_CB_GT = 0.509;
    private final double PURPLE_THRESHOLD_CR_GT = 0.492;
    private final double ARTIFACT_THRESHOLD_CL_GT = 0.01;

    YCbCr frontValues, rightValues, leftValues;
    double frontRed, frontGreen, frontBlue, rightRed, rightGreen, rightBlue, leftRed, leftGreen, leftBlue;

    @Override
    public void postInit() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "intake");
        agitator = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "agitator");
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelLeft");
        rightFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelRight");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hood");
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        rail = ActiveOpMode.hardwareMap().get(Servo.class, "upperRail");
        frontColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "frontColor");
        rightColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "rightColor");
        leftColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "leftColor");
        isIntakeOn = false;

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // =====================================================
    // METHODS
    // =====================================================

    public void runFireWheels() {
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
    }

    public void startRailDex() {
        isShooting = true;
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setTargetPosition(AGITATOR_ENC);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void startRailDexTime() {
        isShooting = true;
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
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

    public void startResetRailDex() {
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
        agitator.setPower(0);
    }

    public void startIntake() {
        isIntakeOn = true;
        intakeMotor.setPower(INTAKE_POWER);
        rail.setPosition(RAIL_UP);
        agitator.setTargetPosition(0);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
    }

    public void floatIntake() {
        intakeMotor.setPower(INTAKE_POWER);
        rail.setPosition(RAIL_UP);
        agitator.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        agitator.setPower(0);
    }

    public void stopIntake() {
        isIntakeOn = false;
        intakeMotor.setPower(INTAKE_POWER_REVERSED);
        intakeRevTimer.reset();
        intakeReversed = true;
    }

    public void stopIntakeNoReverse() {
        isIntakeOn = false;
        intakeMotor.setPower(0);
    }

    public void turnAgitator() {
        agitator.setTargetPosition(agitator.getCurrentPosition() + AGITATOR_ENC / 4);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitatorResetRequest = true;
    }

    public void railDown() {
        rail.setPosition(RAIL_DOWN);
    }

    public void railUp() {
        rail.setPosition(RAIL_UP);
    }

    public void shootAllThree() {
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        isShooting = true;
        railDown();
        startRailDex();
    }

    public void firewheelsOff() {
        isShooting = false;
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
    }

    public void slowIntake() {
        intakeMotor.setPower(INTAKE_SHOOTING_POWER);
    }

    public void fastIntake() {
        intakeMotor.setPower(INTAKE_FAST);
    }

    public void startTransfer() {
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setPower(AGITATOR_POWER);
    }

    public void stopTransfer() {
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
    }

    // =====================================================
    // COLOR SENSOR LOGIC
    // =====================================================

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

        public YCbCr(double luminance, double cBlue, double cRed) {
            cL = luminance;
            cB = cBlue;
            cR = cRed;
        }

        @Override
        public String toString() {
            return "cl:" + String.format("%.4f", cL) + ",cb:" + String.format("%.4f", cB) + ",cr:" + String.format("%.4f", cR);
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

        frontValues.cL = 0.001;
        rightValues.cL = 0.001;
        leftValues.cL = 0.001;
    }

    // =====================================================
    // UPDATE LOOP
    // =====================================================

    public void update() {
        getAllColorSensorValues();
        latchFrontColorSensor();
        latchRightColorSensor();
        latchLeftColorSensor();

        ActiveOpMode.telemetry().addData("front artifact", frontArtifact);
        ActiveOpMode.telemetry().addData("right artifact", rightArtifact);
        ActiveOpMode.telemetry().addData("left artifact", leftArtifact);
        ActiveOpMode.telemetry().addData("left firewheel power", leftFireServo.getPower());
        ActiveOpMode.telemetry().addData("right firewheel power", rightFireServo.getPower());
        ActiveOpMode.telemetry().addData("agitator encoder", agitator.getCurrentPosition());

        if (isShooting && !agitator.isBusy()) {
            isShooting = false;
            agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if (isShooting) {
            leftFireServo.setPower(FIRE_POWER);
            rightFireServo.setPower(FIRE_POWER);
        }

        if (isIntakeOn) {
            resetLatches();
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