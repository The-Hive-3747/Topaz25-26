package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.robotcore.internal.hardware.android.GpioPin;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.subsystems.Subsystem;

import dev.nextftc.ftc.ActiveOpMode;

public class Hood{

    private double HOOD_MAX_POS = 5000;
    private double HOOD_MIN_POS = 0;
    private double HOOD_INCREMENT = 20;
    public static double AUTON_HOOD_POS = 1353;//1453
    private double power = 0;
    private double angle, rotations, offset, lastAngle;
    double hoodAdjust = 0.0;
    public boolean allowPID = true;

    private static double HOOD_P = 0.0006;//0.0012;
    //private static double HOOD_D = 0.014;
    private KineticState goal;

    ControlSystem hoodPID;
    CRServo hood;
    DcMotorEx hoodEncoder;
    public Hood(DcMotorEx encoder){
        hoodEncoder = encoder;
    }

    public void init() {
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        hood.setDirection(DcMotorSimple.Direction.REVERSE);


        // default goal value
        goal = new KineticState(0);

        hoodPID = ControlSystem.builder()
                .posPid(HOOD_P)
                .build();
        hoodPID.setGoal(goal); // set default goal to 0

        allowPID = true;
    }


    public double getHoodPosition() {
        return -hoodEncoder.getCurrentPosition();
    }

    public void setGoal(double goalPos) {
        enableHoodPID();
        goalPos = goalPos + hoodAdjust;
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

    public void increaseHood(){
        hoodAdjust += HOOD_INCREMENT;
        setGoal(getGoal());
    }
    public void decreaseHood(){
        hoodAdjust -= HOOD_INCREMENT;
        setGoal(getGoal());
    }


    public void update() {
        if (allowPID) {
            power = hoodPID.calculate(new KineticState(this.getHoodPosition()));
            if (power > 1) {
                power = 1;
            } else if (power < -1) {
                power = -1;
            }
            hood.setPower(power);
        }

        ActiveOpMode.telemetry().addData("hoodPos", this.getHoodPosition());
        ActiveOpMode.telemetry().addData("hoodAllowed", allowPID);
        ActiveOpMode.telemetry().addData("Hood Goal", getGoal());
        /*
        ActiveOpMode.telemetry().addData("hoodPower", power);
        ActiveOpMode.telemetry().addData("hoodGoal", hoodPID.getGoal());
        ActiveOpMode.telemetry().addData("hoodPos", this.getHoodPosition());*/
    }
}
