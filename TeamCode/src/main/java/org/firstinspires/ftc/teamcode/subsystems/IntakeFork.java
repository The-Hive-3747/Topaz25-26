package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.hardware.PIDCoefficients;
import com.qualcomm.robotcore.hardware.PIDFCoefficients;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;
import org.firstinspires.ftc.teamcode.utilities.Artifact;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class IntakeFork implements Component {
    DcMotor intakeMotor;
    DcMotorEx agitator;
    TurretLights prismLights;
    GoBildaPrismDriver prism;
    NormalizedColorSensor frontColor, rightColor, leftColor;
    ElapsedTime shotTimer = new ElapsedTime();
    ElapsedTime intakeTimer = new ElapsedTime();
    ElapsedTime colorTimer = new ElapsedTime();
    private double COLOR_CHECK_MS = 200;
    static boolean isIntakeOn = false;
    double INTAKE_POWER = 0.9;
    double INTAKE_SHOOTING_POWER = 0.9;
    double INTAKE_FAST = 1.0;
    double REVERSAL_TIME = 500;
    double FIRE_POWER = 1;//0.9
    double AGITATOR_POWER = 0.6; //0.8;//0.2;//0.6;
    public static double RAIL_UP = 0.157;//0.3;//0.8;//0.5
    public static double RAIL_DOWN = 0;//1;//1;
    double INTAKE_POWER_REVERSED = -0.9;
    double agitatorResetPosDone = 0.0;
    double AGITATOR_ENC_REVOLUTIONS_REV_V2 = 8192.0;
    double  AGITATOR_ENC_REVOLUTIONS_GOBILDA_312 = 537.7;
    int AGITATOR_ENC = (int) AGITATOR_ENC_REVOLUTIONS_REV_V2;
    int AGITATOR_FRONT = -AGITATOR_ENC/3;
    int AGITATOR_RIGHT_ONE = AGITATOR_ENC/3;
    int AGITATOR_LEFT_ONE = -AGITATOR_ENC/3;
    int AGITATOR_RIGHT_ALL = AGITATOR_ENC;
    int AGITATOR_LEFT_ALL = -AGITATOR_ENC;
    int AGITATOR_ZERO = 0;

    CRServo leftFireServo, rightFireServo, hood;
    Servo rail;
    ElapsedTime intakeRevTimer = new ElapsedTime();
    boolean intakeReversed = false;
    boolean agitatorResetRequest = false;
    boolean isShooting = false;

    Artifact leftArtifact = Artifact.EMPTY;
    Artifact rightArtifact = Artifact.EMPTY;
    Artifact frontArtifact = Artifact.EMPTY;
    private final double GREEN_THRESHOLD_CB_LT =  0.4994;
    private final double GREEN_MAYBE_THRESHOLD_CR_LT = 0.4999;
    private final double GREEN_THRESHOLD_CR_LT =  0.487;
    private final double PURPLE_THRESHOLD_CB_GT =  0.509;
    private final double PURPLE_THRESHOLD_CR_GT =  0.492;
    private final double ARTIFACT_THRESHOLD_CL_GT = 0.01;

    boolean requestRailUp = false;
    enum IntakeState {
        AUTO_START,
        PREPARING_TO_INTAKE,
        READY_TO_INTAKE,
        FIRST_ARTIFACT_RAIL_DOWN,
        FIRST_ARTIFACT_MOVE,
        INTAKE_OTHER_ARTIFACTS,
        READY_TO_SHOOT,
        SHOOT_LEFT,
        SHOOT_RIGHT,
        SHOOT_LEFT_ALL,
        SHOOT_RIGHT_ALL
    };
    IntakeState intakeState = IntakeState.AUTO_START;
    enum ShootState {
        SHOOT_LEFT,
        SHOOT_RIGHT,
        SHOOT_LEFT_ALL,
        SHOOT_RIGHT_ALL,
        NOT_SHOOTING
    }
    ShootState shootState = ShootState.NOT_SHOOTING;
    boolean isFinishIntaking = false;
    ElapsedTime railTimer = new ElapsedTime();
    double RAIL_DOWN_TIME_MS = 500;
    double INTAKE_OTHER_ARTIFACTS_MS = 1000;
    YCbCr frontValues, rightValues, leftValues;
    double frontRed, frontGreen, frontBlue, rightRed, rightGreen, rightBlue, leftRed, leftGreen, leftBlue;
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();

    ControlSystem agitatorPID;
    public static double KP = 0.0001, KI = 0, KD = 0;


    @Override
    public void postInit() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "intake");
        agitator = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "agitator"); //312 motor with 537.7 pulses per rev
        agitator.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelLeft");
        rightFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelRight");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hood");
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
        rail = ActiveOpMode.hardwareMap().get(Servo.class, "upperRail");
        //rail.setDirection(Servo.Direction.REVERSE);
        frontColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "frontColor");
        rightColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "rightColor");
        leftColor = ActiveOpMode.hardwareMap().get(NormalizedColorSensor.class, "leftColor");
        prism = ActiveOpMode.hardwareMap().get(GoBildaPrismDriver.class, "lights");
        isIntakeOn = false;

        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);//encoder has 8192 pulses per revolution (REV thru V2)

        agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        agitator.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        agitatorPID = ControlSystem.builder()
                .posPid(KP, KI, KD)
                .build();
    }

    public void runIntakeState() {
        switch (intakeState) {
            case AUTO_START:
            case READY_TO_SHOOT:
                switch (shootState) {
                    case NOT_SHOOTING:
                        moveAgitatorToFront();
                        intakeState = IntakeState.PREPARING_TO_INTAKE;
                        break;
                    case SHOOT_LEFT:
                        moveAgitatorLeftOne();
                        intakeState = IntakeState.SHOOT_LEFT;
                        break;
                    case SHOOT_LEFT_ALL:
                        moveAgitatorLeftAll();
                        intakeState = IntakeState.SHOOT_LEFT_ALL;
                        break;
                    case SHOOT_RIGHT:
                        moveAgitatorRightOne();
                        intakeState = IntakeState.SHOOT_RIGHT;
                        break;
                    case SHOOT_RIGHT_ALL:
                        moveAgitatorRightAll();
                        intakeState = IntakeState.SHOOT_RIGHT_ALL;
                        break;
                }
                break;
            case PREPARING_TO_INTAKE:
                if (!agitator.isBusy()) {
                    intakeState = IntakeState.READY_TO_INTAKE;
                    moveRailUp();
                    startIntake();
                    isFinishIntaking = false;
                }
                break;
            case READY_TO_INTAKE:
                if (isFinishIntaking) {
                    intakeState = IntakeState.FIRST_ARTIFACT_RAIL_DOWN;
                    isFinishIntaking = false;
                    stopIntake();
                    railTimer.reset();
                    moveRailDown();
                }
                break;
            case FIRST_ARTIFACT_RAIL_DOWN:
                if (railTimer.milliseconds() > RAIL_DOWN_TIME_MS) {
                    intakeState = IntakeState.FIRST_ARTIFACT_MOVE;
                    moveAgitatorZero();
                }
                break;
            case FIRST_ARTIFACT_MOVE:
                if (!agitator.isBusy()) {
                    moveRailUp();
                    startIntake();
                    intakeState = IntakeState.INTAKE_OTHER_ARTIFACTS;
                    railTimer.reset();
                }
                break;
            case INTAKE_OTHER_ARTIFACTS:
                if (railTimer.milliseconds() > INTAKE_OTHER_ARTIFACTS_MS) {
                    intakeState = IntakeState.READY_TO_SHOOT;
                    moveRailDown();
                }
                break;
            case SHOOT_LEFT:
                if (shootState == ShootState.SHOOT_LEFT_ALL) {
                    moveAgitatorLeftAll();
                    intakeState = IntakeState.SHOOT_LEFT_ALL;
                } else if (shootState == ShootState.SHOOT_RIGHT_ALL) {
                    moveAgitatorRightAll();
                    intakeState = IntakeState.SHOOT_RIGHT_ALL;
                }
                break;
            case SHOOT_RIGHT:
                if (shootState == ShootState.SHOOT_LEFT_ALL) {
                    moveAgitatorLeftAll();
                    intakeState = IntakeState.SHOOT_LEFT_ALL;
                } else if (shootState == ShootState.SHOOT_RIGHT_ALL) {
                    moveAgitatorRightAll();
                    intakeState = IntakeState.SHOOT_RIGHT_ALL;
                }
                break;
            case SHOOT_LEFT_ALL:
                if (!agitator.isBusy()) {
                    intakeState = IntakeState.PREPARING_TO_INTAKE;
                    shootState = ShootState.NOT_SHOOTING;
                    moveAgitatorZero();
                }
                break;
            case SHOOT_RIGHT_ALL:
                if (!agitator.isBusy()) {
                    intakeState = IntakeState.PREPARING_TO_INTAKE;
                    shootState = ShootState.NOT_SHOOTING;
                    moveAgitatorZero();
                }
                break;
        }
    }
    public void finishIntaking() {
        isFinishIntaking = true;
    }
//    public void requestIntakeOn() {
//        isIntakeRequested = false;
//    }

    public void moveAgitatorZero() {
        setAgitatorGoalPosition(AGITATOR_ZERO);
//        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
//        agitator.setTargetPosition(AGITATOR_ZERO);
//        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    public void moveAgitatorLeftOne() {
        setAgitatorGoalPosition(AGITATOR_LEFT_ONE);
//        agitator.setTargetPosition(AGITATOR_LEFT_ONE);
//        agitator.setPower(AGITATOR_POWER);
//        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveAgitatorRightOne() {
        setAgitatorGoalPosition(AGITATOR_RIGHT_ONE);
//        agitator.setTargetPosition(AGITATOR_RIGHT_ONE);
//        agitator.setPower(AGITATOR_POWER);
//        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveAgitatorLeftAll() {
        setAgitatorGoalPosition(AGITATOR_LEFT_ALL);
//        agitator.setTargetPosition(AGITATOR_LEFT_ALL);
//        agitator.setPower(AGITATOR_POWER);
//        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveAgitatorRightAll() {
        setAgitatorGoalPosition(AGITATOR_RIGHT_ALL);
//        agitator.setTargetPosition(AGITATOR_RIGHT_ALL);
//        agitator.setPower(AGITATOR_POWER);
//        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }
    public void moveAgitatorToFront() {
        setAgitatorGoalPosition(AGITATOR_FRONT);
        /*
        agitator.setTargetPosition(AGITATOR_FRONT);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);*/
    }
    public void runFireWheels() {
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
    }
    public void moveRailUp() {
        rail.setPosition(RAIL_UP);
    }
    public void moveRailDown() {
        rail.setPosition(RAIL_DOWN);
    }
    public void setFirewheelsOff() {
        leftFireServo.setPower(0);
        rightFireServo.setPower(0);
    }
    public void startIntake() {
        isIntakeOn = true;
        intakeMotor.setPower(INTAKE_POWER);
        setFirewheelsOff();
    }

    /**
     * sets agitatorPID goal position
     * @param goalPosition in ticks
     */
    public void setAgitatorGoalPosition(double goalPosition) {
        agitatorPID.setGoal(new KineticState(goalPosition));
    }

    public void moveAgitatorToGoal() {
        agitator.setPower(agitatorPID.calculate(new KineticState(agitator.getCurrentPosition())));
    }




    public void startRailDex() {
        isShooting = true;
        leftFireServo.setPower(FIRE_POWER);
        rightFireServo.setPower(FIRE_POWER);
        agitator.setTargetPosition(AGITATOR_ENC);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);

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


    public void stopIntake() {
        isIntakeOn = false;
        intakeMotor.setPower(0);
    }
    public void turnAgitator() {
        agitator.setTargetPosition(agitator.getCurrentPosition() + AGITATOR_ENC/4);
        agitator.setPower(AGITATOR_POWER);
        agitator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        agitatorResetRequest = true;
    }

    public void railDown(){
        rail.setPosition(RAIL_DOWN);
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
    public Command railDownAuto = new InstantCommand(
            () -> rail.setPosition(RAIL_DOWN)
    );
    public Command shootAllThree = new LambdaCommand()
            .setStart(() ->{
                leftFireServo.setPower(FIRE_POWER);
                rightFireServo.setPower(FIRE_POWER);
                shotTimer.reset();
                isShooting = true;
                railDown();
                startRailDex();
            })
            .setStop(interrupted -> {})
            //in order to reset agitator fully it needs 2.75 seconds but its usually covered in the driving.
            .setIsDone(() -> (shotTimer.seconds() > 2.25)); //2 2.2 2

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


    public void update() {
        agitatorPID = ControlSystem.builder()
                .posPid(KP, KI, KD)
                .build();

        if(colorTimer.milliseconds() > COLOR_CHECK_MS) {
            getAllColorSensorValues();
            latchFrontColorSensor();
            latchRightColorSensor();
            latchLeftColorSensor();
            colorTimer.reset();
        }

        /*
        panelsTelemetry.addData("front artifact", frontArtifact);
        panelsTelemetry.addData("right artifact", rightArtifact);
        panelsTelemetry.addData("left artifact", leftArtifact);
        ActiveOpMode.telemetry().addData("front artifact", frontArtifact);
        ActiveOpMode.telemetry().addData("right artifact", rightArtifact);
        ActiveOpMode.telemetry().addData("left artifact", leftArtifact);
        ActiveOpMode.telemetry().addData("left firewheel power", leftFireServo.getPower());
        ActiveOpMode.telemetry().addData("right firewheel power", rightFireServo.getPower());*/
        ActiveOpMode.telemetry().addData("intake state", intakeState);
        ActiveOpMode.telemetry().addData("agitator encoder", agitator.getCurrentPosition());
        /*ActiveOpMode.telemetry().addData("front color", Color.luminance(Color.rgb(frontColor.red(), frontColor.green(), frontColor.blue())));
        //ActiveOpMode.telemetry().addData("right color", rightColor.red());
        //ActiveOpMode.telemetry().addData("left color", leftColor.red());*/


        runIntakeState();
        moveAgitatorToGoal();

        if (isShooting && !agitator.isBusy()){
            isShooting = false;
            agitator.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        }

        if(isShooting){
            leftFireServo.setPower(FIRE_POWER);
            rightFireServo.setPower(FIRE_POWER);
        }

        if (isIntakeOn){
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
}
