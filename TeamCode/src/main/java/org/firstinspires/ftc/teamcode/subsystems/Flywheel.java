package org.firstinspires.ftc.teamcode.subsystems;


import com.bylazar.configurables.annotations.Configurable;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

@Configurable
// This is a component file for the flywheel / shooter.
public class Flywheel implements Component {

    DcMotorEx flywheelBottom, flywheelTop, intakeMotor;
    static double correct, flywheelVel, targetVel, currentRPM;
    static int countPerRevolution = 8192;
    public double convertedVel;
    double currentPosition, pastPosition = 0, currentTime, pastTime = 0, deltaTime, deltaPosition;
    private ElapsedTime shotTimer = new ElapsedTime();
    private ElapsedTime flywheelVelocityTimer = new ElapsedTime();
    ControlSystem largeFlywheelPID;
    Servo flipper;
    CRServo leftFireServo, sideWheelServo;
    Hood hood;
    double autoTargetVel = 2200; //UPDATED TO RPM
    public static double FLYWHEEL_PID_KP = 0.002655, FLYWHEEL_PID_KV = 0.000245, FLYWHEEL_PID_KS = 0.135, FLYWHEEL_PID_KD = 1, FLYWHEEL_PID_KI = 0;
    double targetAdjust = 0;
    double READY_VEL_THRESHOLD = 200; // UPDATED TO RPM
    public static double AUTON_SHOOT_VEL = 2200; //UPDATED TO RPM
    @Override
    public void postInit() { // this runs AFTER the init, it runs just once
        //this needs to be forward in order to use the hood PID. correction is in set power
        flywheelBottom = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelBottom");
        flywheelBottom.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelTop = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelTop");
        flywheelTop.setDirection(DcMotorEx.Direction.FORWARD);
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "transfer");
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "left_firewheel");
        leftFireServo.setDirection(DcMotorSimple.Direction.REVERSE);
        sideWheelServo = ActiveOpMode.hardwareMap().get(CRServo.class, "side-wheel");

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelBottom.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
        flywheelTop.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);


        flywheelTop.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        flywheelTop.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);


        flipper = ActiveOpMode.hardwareMap().get(Servo.class, "flipper");
        hood = new Hood(intakeMotor);
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
        intakeMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void increaseHood() {
        hood.increaseHood();
    }

    public void decreaseHood(){
        hood.decreaseHood();
    }

    public void increase(){
        targetAdjust += 5;
        double targetV = targetVel;
        if (targetVel + targetAdjust < 0){
            targetV = 0;
        }else{
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }

    public void decrease(){
        targetAdjust -= 5;
        double targetV = targetVel;
        if (targetVel + targetAdjust < 0){
            targetV = 0;
        }else{
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }

    public boolean readyToShoot(){
        return Math.abs(targetVel + targetAdjust - getVel()) <= READY_VEL_THRESHOLD;
    }

    /**
     *
     * @param power sets motor power. DONT use this method normally, its not smart
     */
    public void setPower(double power) {
        //this is to correct the flywheel direction
        flywheelBottom.setPower(power);
        flywheelTop.setPower(power);
    }

    /**
     *
     * @return double: current in milliamps
     */
    public double getCurrent() {
        return flywheelTop.getCurrent(CurrentUnit.MILLIAMPS) + flywheelBottom.getCurrent(CurrentUnit.MILLIAMPS);
    }


    /**
     *
     * @return motor power
     */
    public double getPower() {
        return flywheelTop.getPower();

    }


    /**
     *
     * @return gets flywheel motor velocity
     */
    public double getVel() {
        // unfortunately the encoder we're using is extremely chopped and reads random negative values
        // when using flywheelTop.getVelocity().
        // instead, we're manually calculating the flywheel velocity by getting the distance & time
        // delta distance / delta time = velocity
        // (delta means difference between)

        currentTime = flywheelVelocityTimer.seconds();
        currentPosition = flywheelTop.getCurrentPosition();
        deltaPosition = currentPosition - pastPosition;
        deltaTime = currentTime - pastTime;

        //convertedVel = (flywheelTop.getVelocity()/countPerRevolution) * 60; //to convert to RPM (60 for seconds)
        convertedVel = ((deltaPosition / deltaTime)/countPerRevolution) * 60; //to convert to RPM (60 for seconds)

        pastPosition = currentPosition;
        pastTime = currentTime;

        return convertedVel;
    }


    /**
     *
     * @param vel: sets target velocity
     */
    public void setTargetVel(double vel) {
        double targetV = vel;
        targetVel = vel;
        if (targetVel + targetAdjust < 0){
            targetV = 0;
        }else{
            targetV = targetVel + targetAdjust;
        }
        largeFlywheelPID.setGoal(new KineticState(0, targetV));
    }




    // simple update function. telling the controller the robot's current velocity, and it returns a motor power
    public void update() {
        flywheelVel = this.getVel();

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

        //if(Math.abs(targetVel+targetAdjust-flywheelVel)>11) {
        //    this.setPower(correct); // set the motor power!
        //}
        this.setPower(correct);

        hood.update();

        ActiveOpMode.telemetry().addData("flywheel power", correct);
        ActiveOpMode.telemetry().addData("flywheel vel", flywheelVel);
        ActiveOpMode.telemetry().addData("flywheel target vel", targetVel + targetAdjust);
        ActiveOpMode.telemetry().addData("Adjust Target By",targetAdjust);
    }

    /**
     *
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
    public Command startFlywheel = new InstantCommand(
            () -> this.setTargetVel(autoTargetVel)
    );


    public Command stopFlywheel = new InstantCommand(
            () -> this.setTargetVel(0)
    );

    public Command resetShotTimer = new InstantCommand(
            () ->  shotTimer.reset()
    );


    public Command shootAllThree = new LambdaCommand()
            .setUpdate(() -> {
                if (shotTimer.seconds() > 1.7){//1.5 1.3 1.0
                    flipper.setPosition(0.1);
                    return;
                }
                currentRPM = this.getVel();
                if (this.readyToShoot()) {  //was 200
                    flipper.setPosition(0.1);
                } else {
                    flipper.setPosition(0.52);
                }
            })
            .setStop(interrupted -> {
                flipper.setPosition(0.52);
            })
            .setIsDone(() -> (shotTimer.seconds() > 2.3)); //2.2 2
}
