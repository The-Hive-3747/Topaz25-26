package org.firstinspires.ftc.teamcode.utilities;

import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.Turret;


import java.io.File;
import java.io.FileWriter;
import java.io.IOException;
import java.text.SimpleDateFormat;
import java.util.Date;

public class DataLogger implements Component{

    DcMotorEx flywheelLeft, flywheelRight;
    DcMotor agitator;
    CRServo hood;
    Pose currentPose = OpModeTransfer.currentPose;
    Alliance alliance;
    String logEntry;
    double goalX, goalY, botDistance, flywheelVelocity, hoodPos;
    Pose botPosition;
    double timeShooting;
    private ElapsedTime AgitatorTime;
    private boolean isAgitatorDone = false;

    private boolean isAgitatorRun = false;
    private Telemetry telemetry;
    private FileWriter dataWriter;
    private File dataLog;
    private Aimbot aimbot;
    private Turret turret;

    String timeStamp;
    public DataLogger(Telemetry telemetry){
        this.telemetry = telemetry;
    }
    @Override
    public void postInit() {
        flywheelRight = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelRight"); //for hood position
        agitator = ActiveOpMode.hardwareMap().get(DcMotor.class, "agitator"); //312 motor with 537.7 pulses per rev
        flywheelLeft = ActiveOpMode.hardwareMap().get(DcMotorEx.class, "flyWheelLeft"); //for flywheel vel


        createCSVFile();
        AgitatorTime = new ElapsedTime();
    }

    public void update() {
        botPosition = this.currentPose;
        botDistance = aimbot.getBotDistance();
        flywheelVelocity = flywheelLeft.getVelocity();
        //hoodPos = -intake.getCurrentPosition();
        hoodPos = -flywheelRight.getCurrentPosition();

        ActiveOpMode.telemetry().addData("Bot Position",botPosition); //need to put follower in current pose in update of opmode
        ActiveOpMode.telemetry().addData("Distance to Goal",botDistance);
        ActiveOpMode.telemetry().addData("Flywheel vel",flywheelVelocity);
        ActiveOpMode.telemetry().addData("Flywheel goal vel","uhh");
        ActiveOpMode.telemetry().addData("hood pos", hoodPos); //hood encoder is on intake
        ActiveOpMode.telemetry().addData("last time shooting",timeShooting);
        ActiveOpMode.telemetry().update();

        if (agitator.isBusy() && !isAgitatorRun && !isAgitatorDone) {
            AgitatorTime.reset();
            isAgitatorRun = true;

        }
        else if (isAgitatorRun && !isAgitatorDone && agitator.isBusy() && aimbot!=null) {
            timeStamp = new SimpleDateFormat("yyyyMMdd_HHmmss").format(new Date());
            timeShooting = AgitatorTime.milliseconds();
            try {
                dataWriter.write(String.format(
                        "%s, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f, %.1f\n",
                        timeStamp,
                        botPosition.getX(),
                        botPosition.getY(),
                        botPosition.getHeading(),
                        botDistance,
                        flywheelVelocity,
                        hoodPos,
                        timeShooting,
                        aimbot.getAimVelocity(),
                        aimbot.getAimHoodPos(),
                        turret.getTurretAngle()
                        ));
                isAgitatorDone = true;
                dataWriter.flush();
            } catch (IOException e) {
                throw new RuntimeException(e);
            }

        }else if (!agitator.isBusy() && isAgitatorDone && isAgitatorRun){
            isAgitatorDone = false;
            isAgitatorRun = false;


        }
    }
    public void addAimbot(Aimbot aimbot) {
        this.aimbot = aimbot;
    }
    public void addTurret(Turret turret) {
        this.turret = turret;
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
