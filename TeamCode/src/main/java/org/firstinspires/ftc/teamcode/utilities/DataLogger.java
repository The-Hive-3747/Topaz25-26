package org.firstinspires.ftc.teamcode.utilities;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.telemetry;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.utilities.BotStats;
import org.firstinspires.ftc.teamcode.subsystems.Aimbot;


import org.firstinspires.ftc.robotcore.internal.opmode.TelemetryImpl;
import org.firstinspires.ftc.teamcode.subsystems.Hood;

import java.io.File;
import java.io.FileWriter;
import java.io.IOException;

public class DataLogger implements Component{

    DcMotorEx flywheelBottom, intake;
    CRServo hood;
    Servo flipper;
    Pose currentPose = OpModeTransfer.currentPose;
    Alliance alliance;
    String logEntry;
    double goalX, goalY, botDistance, flywheelVelocity, hoodPos;
    Pose botPosition;
    double timeShooting;
    private ElapsedTime flipperTime;
    private boolean isFlipperOn;
    private Telemetry telemetry;
    private FileWriter dataWriter;
    private File dataLog;
    public double aimbotVel, aimbotHood, turretAngle;
    public DataLogger(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    public BotStats botStats(double aimVel, double aimHoodAng, double turretAng) {
        this.aimbotVel = aimVel;
        this.aimbotHood = aimHoodAng;
        this.turretAngle = turretAng;
        return new BotStats(aimVel, aimHoodAng, turretAng);
    }
    @Override
    public void postInit() {
        flywheelBottom = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flywheelBottom");
        intake = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "transfer");
        flipper = ActiveOpMode.hardwareMap().get(Servo.class, "flipper");

        createCSVFile();
        flipperTime = new ElapsedTime();
    }

    public void update() {
        botPosition = this.currentPose;
        botDistance = this.getBotDistance();
        flywheelVelocity = flywheelBottom.getVelocity();
        hoodPos = -intake.getCurrentPosition();

        ActiveOpMode.telemetry().addData("Bot Position",botPosition); //need to put follower in current pose in update of opmode
        ActiveOpMode.telemetry().addData("Distance to Goal",botDistance);
        ActiveOpMode.telemetry().addData("Flywheel vel",flywheelVelocity);
        ActiveOpMode.telemetry().addData("Flywheel goal vel","uhh");
        ActiveOpMode.telemetry().addData("hood pos", hoodPos); //hood encoder is on intake
        ActiveOpMode.telemetry().addData("last time shooting","timeShooting");
        ActiveOpMode.telemetry().update();

        if (flipper.getPosition()==0.1 && !isFlipperOn) {
            flipperTime.reset();
            isFlipperOn = true;

        }
        else if (flipper.getPosition()!=0.1 && isFlipperOn) {
            timeShooting = flipperTime.milliseconds();
            try {
                dataWriter.write(String.format(
                        "%.1f, %.1f, %.1f, %.1f, %.1f %.1f, %.1f\n",
                        botPosition.getX(),
                        botPosition.getY(),
                        botPosition.getHeading(),
                        botDistance,
                        flywheelVelocity,
                        hoodPos,
                        timeShooting
                        ));
                dataWriter.flush();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }
            //logEntry.writeLine();
            isFlipperOn = false;
        }







    }

    public void setAlliance(Alliance all) {
        this.alliance = all;
        if (this.alliance == Alliance.RED) {
            goalX = 129; //this is the point of the middle of the front panel of the goal
            goalY = 129;
        } else {
            goalX = 15;
            goalY = 129;
        }
    }
    public double getBotDistance() {
        //this is math for the distance from bot to goal using hypotenuse of x and y
        botDistance = Math.sqrt(Math.pow(goalX - currentPose.getX(), 2) + Math.pow(goalY - currentPose.getY(), 2));
        return botDistance;
    }
    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }
    public void createCSVFile() {
        try {
            File directory = new File("/sdcard/AimbotLog");
            if (!directory.exists()) {
                directory.mkdirs(); //if the directory doesn't exist, it creates the directories
            }
            dataLog = new File(directory, "dataLogger.csv");

            dataWriter = new FileWriter(dataLog, true);
        } catch (IOException e) { //if error shows up
            telemetry.addData("Error", "Failed to create CSV:" + e.getMessage());
        }
    }


}
