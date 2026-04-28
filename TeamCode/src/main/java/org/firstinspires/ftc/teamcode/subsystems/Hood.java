package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.control.feedforward.FeedforwardElement;

import com.bylazar.configurables.annotations.Configurable;
import com.bylazar.telemetry.PanelsTelemetry;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;

import dev.nextftc.ftc.ActiveOpMode;

@Configurable
public class Hood{

    private static double HOOD_MAX_POS = 12000;//9000;//8858;
    private double HOOD_MIN_POS = 0;
    public static double HOOD_INCREMENT = 200;//1000;
    public static double AUTON_HOOD_POS = 1353;//1453
    private double power = 0;
    double hoodOffset = 0.0;
    public boolean allowPID = true;

    public static double HOOD_P = 0.0002;//0.00047;//0.0001;//0.00029;//0.00058;//0.0012;
    public static double HOOD_D = 0;//0.1;//0;//0.3;//0.1;//0;//1;
    public static double HOOD_I = 0;//0.00000000000000001;//0.0000000000003;//0.0;
    public static double HOOD_FF = 0;

    public static double HOOD_AUTON_CLOSE_POS = 5700;//5500;//5250;
    public static double HOOD_AUTON_FAR_POS = 8150; //8300; //9200 7500
    public static double HOOD_MANUAL_POWER = 0.3;
    private KineticState goal;
    ControlSystem hoodPID;
    CRServo hood;
    DcMotorEx hoodEncoder;
    private int hoodPositionCached=0;
    public Hood(DcMotorEx encoder){
        hoodEncoder = encoder;
    }
    ElapsedTime resetHoodTimer = new ElapsedTime();
    double HOOD_RESET_TIME_MS = 2000;
    boolean isHoodResetting = false;
    double HOOD_RESET_POWER = -1;
    static boolean manualMode = false;

    public class FrictionFeedback implements FeedforwardElement {
        double HOOD_FF_INNER = 0;
        public FrictionFeedback(double ff){
            HOOD_FF_INNER = ff;
        }
        @Override
        public double calculate(KineticState reference){
            return Math.signum(reference.getVelocity()) * HOOD_FF_INNER;
        }
        @Override
        public void reset(){
        }
    }

    public void init() {
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hood");
        hood.setDirection(DcMotorSimple.Direction.REVERSE);

        // default goal value
        goal = new KineticState(0);

        hoodPID = ControlSystem.builder()
                .posPid(HOOD_P, HOOD_I, HOOD_D)
                //.basicFF(HOOD_FF)
                //.feedforward(new FrictionFeedback(HOOD_FF))
                .build();
        hoodPID.setGoal(goal); // set default goal to 0

        allowPID = true;
    }

    //Use the cached value, update once in the update function.
    public double getHoodPosition() {
        return hoodPositionCached; //hoodEncoder.getCurrentPosition();
    }

    public void setGoal(double goalPos) {
        enableHoodPID();
        goalPos = goalPos + hoodOffset;
        if (goalPos > HOOD_MAX_POS) {
            goalPos = HOOD_MAX_POS;
        } else if (goalPos < HOOD_MIN_POS) {
            goalPos = HOOD_MIN_POS;
        }
        goal = new KineticState(goalPos);
        hoodPID.setGoal(goal);
    }

    public double getGoal() {
        return goal.component1();
    }

    public void setHoodPower(double hoodPower) {
        allowPID = false;
        hood.setPower(hoodPower);
    }
    public void enableHoodPID() {
        allowPID = true;
    }
    public static void enableManualMode() {
        manualMode = true;
    }
    public static void disableManualMode() {
        manualMode = false;
    }

    /**
     * Use to apply a permanent offset to all hood goals.
     * Should only be used if the hood is consistently too low/high in a match.
     * @param offset The offset to be added to the existing offset (can be negative)
     */
    public void adjustHoodOffset(double offset) {
        hoodOffset += offset;
    }

    public void increaseHood(){
        setGoal(getGoal() + HOOD_INCREMENT);
    }
    public void decreaseHood(){
        setGoal(getGoal() - HOOD_INCREMENT);
    }

    /**
     * Use to reset the hood position using a timer.
     * Runs the hood with negative power for a time, then resets the encoder.
     */
    public void resetHoodPosUsingTimer() {
        isHoodResetting = true;
        resetHoodTimer.reset();
        setHoodPower(HOOD_RESET_POWER);
    }

    public void update() {
        //update all the cached values, only get the position once
        hoodPositionCached=hoodEncoder.getCurrentPosition();

        //uncomment this if working on the PID otherwise it won't update the constants every loop
        /*
        hoodPID = ControlSystem.builder()
                .posPid(HOOD_P, HOOD_I, HOOD_D)
                //.basicFF(HOOD_FF)
                //.feedforward(new FrictionFeedback(HOOD_FF))
                .build();
        hoodPID.setGoal(goal);*/

        if (isHoodResetting) {
            allowPID = false;
            if (resetHoodTimer.milliseconds() > HOOD_RESET_TIME_MS) {
                hood.setPower(0);
                hoodEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                hoodEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
                isHoodResetting = false;
                allowPID = true;
            }
        }

        if (allowPID && !manualMode) {
            //calculate the power we need and then add a feedforward in order to overcome friction
            power = hoodPID.calculate(new KineticState(this.getHoodPosition()));
            power = power + Math.signum(power) * HOOD_FF;
            if (power > 1) {
                power = 1;
            } else if (power < -1) {
                power = -1;
            }
            hood.setPower(power);
        }
    }

    public void telemetry() {
        PanelsTelemetry.INSTANCE.getTelemetry().addData("hood power", power);
        ActiveOpMode.telemetry().addData("hoodPos", this.getHoodPosition());
        ActiveOpMode.telemetry().addData("hoodAllowed", allowPID);
        ActiveOpMode.telemetry().addData("Hood Goal", getGoal());
    }
}
