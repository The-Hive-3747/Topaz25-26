package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
// This is a component file for the flywheel / shooter.
public class Flywheel implements Component {


    DcMotorEx flywheelRight, flywheelLeft;
    static double correct, flywheelVel, targetVel;
    static int TICKS_PER_REVOLUTION_REV_THRU = 8192;
    static int TICKS_PER_REVOLUTION_MELONBOTICS_THRU = 4096;
    static int TICKS_PER_REVOLUTION_GOBUILDA_YELLOWJACKET_6000_RPM = 28;
    static int TICKS_PER_REVOLUTION = TICKS_PER_REVOLUTION_MELONBOTICS_THRU;
    public double convertedVel;
    double currentVel;
    boolean USE_VEL_CALC_RPM = true;
    double currentPosition, pastPosition = 0, currentTime, pastTime = 0, deltaTime, deltaPosition;
    private final ElapsedTime flywheelVelocityTimer = new ElapsedTime();
    ControlSystem largeFlywheelPID;
    CRServo leftFireServo;
    Hood hood;
    public static double FLYWHEEL_AUTO_TARGET_VEL_FRONT = 2800; //UPDATED TO RPM
    public static double FLYWHEEL_AUTO_TARGET_VEL_BACK = 3900; //UPDATED TO RPM
    public static double FLYWHEEL_PID_KP = 0.00055;
    public static double FLYWHEEL_PID_KV = 0.00018;//0.000245;
    public static double FLYWHEEL_PID_KS = 0.07; //JEM: 0.05;//0.135;
    public static double FLYWHEEL_PID_KD = 1;
    public static double FLYWHEEL_PID_KI = 0;
    public static boolean USE_BANG_BANG = true;
    double targetAdjust = 0;
    double READY_VEL_THRESHOLD = 200; // UPDATED TO// RPM

    @Override
    public void postInit() { // this runs AFTER the init, it runs just once
        //this needs to be forward in order to use the hood PID. correction is in set power
        flywheelRight = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelRight");
        flywheelRight.setDirection(DcMotorSimple.Direction.FORWARD);
        flywheelLeft = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelLeft");
        flywheelLeft.setDirection(DcMotorEx.Direction.REVERSE);

        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "fireWheelLeft");
        leftFireServo.setDirection(DcMotorSimple.Direction.REVERSE);

        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        flywheelRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        flywheelRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood = new Hood(flywheelRight);
        hood.init();

        // a control system is NextFTC's way to build.. control systems!
        largeFlywheelPID = ControlSystem.builder()
                .basicFF(FLYWHEEL_PID_KV, 0, FLYWHEEL_PID_KS) // we use a FeedForward (which pushes hard)
                .velPid(FLYWHEEL_PID_KP, FLYWHEEL_PID_KI, FLYWHEEL_PID_KD) // and a velocity PID (for fine tuning)
                .build(); // and build the control system!

        targetVel = 0; // setting a target velocity of 0 so that the robot doesnt blow up on start

        flywheelVelocityTimer.reset();
    }


    /**
     * self-explanatory, resets the hood encoder
     */
    public void resetHoodEncoder() {
        flywheelLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void increaseHood() {
        hood.increaseHood();
    }

    public double getHoodPosition() {
        return hood.getHoodPosition();
    }

    public void decreaseHood() {
        hood.decreaseHood();
    }

    public void increase() {
        targetAdjust += 50; //5
        double targetV = targetVel;
        if (targetVel + targetAdjust < 0) {
            targetV = 0;
        } else {
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }

    public void decrease() {
        targetAdjust -= 50; //5
        double targetV = targetVel;
        if (targetVel + targetAdjust < 0) {
            targetV = 0;
        } else {
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }

    public boolean readyToShoot() {
        return Math.abs(targetVel + targetAdjust - getVel()) <= READY_VEL_THRESHOLD;
    }

    /**
     *
     * @param power sets motor power. DONT use this method normally, its not smart
     */
    public void setPower(double power) {
        //this is to correct the flywheel direction
        flywheelRight.setPower(power);
        flywheelLeft.setPower(power);
    }

    /**
     *
     * @return double: current in milliamps
     */
    public double getCurrent() {
        return flywheelLeft.getCurrent(CurrentUnit.MILLIAMPS) + flywheelRight.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getCurrentLeft() {
        return flywheelLeft.getCurrent(CurrentUnit.MILLIAMPS);
    }

    public double getCurrentRight() {
        return flywheelRight.getCurrent(CurrentUnit.MILLIAMPS);
    }


    /**
     *
     * @return motor power
     */
    public double getPower() {
        return flywheelRight.getPower();

    }


    /**
     *
     * @return gets flywheel motor velocity
     */
    private void getVelCache() {
        //NOTE: We are caching the basis for this value (currentVel). Do not read more than you need to.
        //TODO: Note that a REV Encoder turning at high speed exceeds the capability of the 16 bit
        // velocity register, due to its large (8k) number of ticks per rotation. If you use a
        // REV encoder, you will need to use the position register, which has 32 bits of resolution
        // and then calculate a speed. However, this will average speed over your critical loop time
        // and may not be as responsive as desired.
        // unfortunately the encoder we're using is extremely chopped and reads random negative values
        // when using flywheelTop.getVelocity().
        // instead, we're manually calculating the flywheel velocity by getting the distance & time
        // delta distance / delta time = velocity
        // (delta means difference between)

        if(USE_VEL_CALC_RPM) {
            currentTime = flywheelVelocityTimer.seconds();
            currentPosition = flywheelLeft.getCurrentPosition();
            deltaPosition = currentPosition - pastPosition;
            deltaTime = currentTime - pastTime;

            convertedVel = -((deltaPosition / deltaTime) / TICKS_PER_REVOLUTION) * 60; //to convert to RPM (60 for seconds)

            pastPosition = currentPosition;
            pastTime = currentTime;


            return;
        }
        convertedVel = -(currentVel / TICKS_PER_REVOLUTION) * 60;
        return;
    }

    public double getVel(){
        return convertedVel;
    }

    /**
     *
     * @param vel: sets target velocity
     */
    public void setTargetVel(double vel) {
        double targetV = vel;
        targetVel = vel;
        if (targetVel + targetAdjust < 0) {
            targetV = 0;
        } else {
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }


    // simple update function. telling the controller the robot's current velocity, and it returns a motor power
    public void update() {
        //cache this value so we don't read it multiple times
        currentVel = flywheelLeft.getVelocity();
        getVelCache();

        //start update calculations
        flywheelVel = this.getVel();


        if(!USE_BANG_BANG) {
            // correct is the motor power we need to set!
            correct = largeFlywheelPID.calculate( // calculate() lets us plug in current vals and outputs a motor power
                    new KineticState(0, flywheelVel) // a KineticState is NextFTC's way of storing position, velocity, and acceleration all in one variable
            );

            // setting constraints on our motor power so its not above 1 and not below 0
            if (targetVel != 0) {
                if (correct < 0) {
                    correct = 0;
                }
                correct = Math.min(0.9, correct);
            } else {
                correct = 0;
            }
        }else{ // USE BANG BANG
            if(targetVel > flywheelVel && targetVel > 0){
                correct = 0.9;
            }else{
                correct = 0;
            }
        }

        this.setPower(correct);

        hood.update();

        ActiveOpMode.telemetry().addData("flywheel power", correct);
        ActiveOpMode.telemetry().addData("flywheel vel", flywheelVel);
        ActiveOpMode.telemetry().addData("flywheel target vel", targetVel + targetAdjust);
        ActiveOpMode.telemetry().addData("Adjust Target By", targetAdjust);
    }

    /**
     * @return flywheel goal velocity, in rpm
     */
    public double getFlywheelGoal() {
        return largeFlywheelPID.getGoal().getVelocity();
    }


    // q: why does the hood own the flywheel?
    // a: the hood encoder uses the left flywheel's encoder port,
    // so everything becomes easier when the hood is owned by the flywheel

    // HOOD METHODS
    public double getHoodPos() {
        return hood.getHoodPosition();
    }

    public void setHoodGoalPos(double pos) {
        hood.setGoal(pos);
    }

    public double getHoodGoal() {
        return hood.getGoal();
    }

    public void setHoodPower(double pow) {
        hood.setHoodPower(pow);
    }

    public void enableHoodPid() {
        hood.enableHoodPID();
    }

    public Command startFlywheelFront = new InstantCommand(
            () -> this.setTargetVel(FLYWHEEL_AUTO_TARGET_VEL_FRONT)
    );

    public Command startFlywheelBack = new InstantCommand(
            () -> this.setTargetVel(FLYWHEEL_AUTO_TARGET_VEL_BACK)
    );

    public Command stopFlywheel = new InstantCommand(
            () -> this.setTargetVel(0)
    );
}