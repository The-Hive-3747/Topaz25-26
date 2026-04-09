package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.NextFTCOpMode;

public abstract class AutoTemplate extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                turret = new Turret()
                //light = new Light()
        );
    }
    protected CommandGroup autonomousCommands;
    protected Alliance alliance = Alliance.BLUE; // default value
    protected Pose startPose;
    public static Pose lastPose;
    ElapsedTime initTimer = new ElapsedTime();
    double timeToInit = 0;
    Turret turret;
    Flywheel flywheel;
    Intake intake;
    TelemetryManager telemetryM;
    Follower follower;
    double HOOD_POS;
    boolean FIREWHEELS_ON = false;


    /**
     * An abstract method to be used by child classes in order to init stuff in auto.
     * Should override alliance, autonomousCommands, and startPose
     */
    public abstract void initAuto();


    @Override
    public void onInit() {
        customParkPose = false;
        initTimer.reset();
        autonomousCommands = new SequentialGroup(
                new InstantCommand(() -> telemetry.addLine())
        );

        follower = Constants.createFollower(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        initAuto();

        AutoPaths.generatePaths(follower);

        follower.setStartingPose(startPose);
        follower.update();

        //turretLights = new TurretLights(hardwareMap, telemetry);

        // TODO: ADD TURRET LIGHTS
        /*
        if (alliance == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
        } */

        turret.zeroTurret();
        timeToInit = initTimer.seconds();
    }

    @Override
    public void onWaitForStart() {
        turret.setCurrentPose(follower.getPose(), follower.getVelocity(), 0);
        turret.setTurretStateFixed();
        turret.update();

        flywheel.setHoodGoalPos(HOOD_POS);
        flywheel.setTargetVel(0);
        flywheel.update();

        telemetry.addData("time to init", timeToInit);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("alliance", alliance);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousCommands.schedule();
    }
    @Override
    public void onUpdate() {
        turret.setTurretStateFixed();
        turret.update();
        follower.update();

        flywheel.setHoodGoalPos(HOOD_POS);

        Drawing.drawOnlyCurrent(follower);

        if (FIREWHEELS_ON) {
            intake.runFireWheels();
        }

        telemetry.addData("pose", follower.getPose());

        flywheel.update();
        intake.update();
        telemetry.update();
    }

    @Override
    public void onStop() {
        // transfer everything
        OpModeTransfer.currentPose = follower.getPose();
        OpModeTransfer.alliance = alliance;
        OpModeTransfer.hasBeenTransferred = true;
    }

    protected void setTurretFixedClose() {
        turret.setAlliance(alliance);
        turret.setFixedAngleClose(alliance);
    }

    protected void setTurretFixedFar() {
        turret.setAlliance(alliance);
        turret.setFixedAngleFar(alliance);
    }

    protected void setHoodPosClose() {
        HOOD_POS = Hood.HOOD_AUTON_CLOSE_POS;
    }

    protected void setHoodPosFar() {
        HOOD_POS = Hood.HOOD_AUTON_FAR_POS;
    }

    protected void startAsBlue() {
        alliance = Alliance.BLUE;
        AutoPaths.alliance = alliance;
        turret.setAlliance(alliance);
        turret.setFixedAngleClose(alliance);
    }

    protected void startAsRed() {
        alliance = Alliance.RED;
        AutoPaths.alliance = alliance;
        turret.setAlliance(alliance);
        turret.setFixedAngleClose(alliance);
    }

    protected void startAtBack() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(79, 9.5, Math.toRadians(0));
        } else {
            startPose = new Pose(63.25, 9.5,Math.toRadians(180));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void startAtFront() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(110, 134.25, Math.toRadians(-94.95));
        } else {
            startPose = new Pose(34, 135.5,Math.toRadians(-85.05));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void startAtCustomPose(Pose pose) {
        startPose = pose;
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void firewheelsOff() {
        intake.turnIsShootingFalse();
    }
    protected void delay(double time) {
        autonomousCommands = autonomousCommands.then(
                new Delay(time)
        );
    }

    protected void turnFlywheelOnForFront() {
        autonomousCommands = autonomousCommands.then(flywheel.startFlywheelFront);
    }

    protected void turnFlywheelOnForBack() {
        autonomousCommands = autonomousCommands.then(flywheel.startFlywheelBack);
    }

    protected void turnFlywheelOnCustom(double power) {
        autonomousCommands = autonomousCommands.then(
                new InstantCommand(() -> flywheel.setTargetVel(power))
        );
    }

    protected void shootAllThreeAtClose(double delayBeforeShot) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
            new ParallelGroup(
                new FollowPath(toShootAtCloseFromLastPose),
                intake.railDownAuto
            ),
                new Delay(delayBeforeShot),
                new ParallelGroup(
                        new InstantCommand(() -> FIREWHEELS_ON=true),
                        new InstantCommand(() -> intake.turnIsShootingTrue()),
                        intake.shootAllThree
                )
        ));
        lastPose = closeShootingPose;
    }

    protected void shootAllThreeAtCloseCurved(double delayBeforeShot) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(toShootAtCloseFromLastPoseCurved),
                        intake.railDownAuto
                ),
                new Delay(delayBeforeShot),
                new ParallelGroup(
                        new InstantCommand(() -> intake.turnIsShootingTrue()),
                        intake.shootAllThree
                )
        ));
        lastPose = closeShootingPose;
    }

    protected void shootAllThreeAtFar(double delayBeforeShot) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(toShootAtFarFromLastPose),
                        intake.railDownAuto
                ),
                new Delay(delayBeforeShot),
                new ParallelGroup(
                        new InstantCommand(() -> FIREWHEELS_ON=true),
                        new InstantCommand(() -> intake.turnIsShootingTrue()),
                        intake.shootAllThree
                )
                // idk what this does commenting it out fr -neset
                /*,
                intake.railUpAuto,
                //this delay is so we can get the rail up in time to reverse any extra balls out
                new Delay(0.2),
                intake.reverseIntake,
                //this delay is for the time of reversing the intake
                new Delay(0.4),
                intake.stopIntake*/
        ));
        lastPose = farShootingPose;
    }
    //use this in case a ball gets stuck in the shooting mechanism
    /*protected void flywheelJiggle(){
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new FollowPath(farJigglePath),
                new FollowPath(toShootAtFarFromLastPose)
        ));
    }*/
    protected void shootAllThreeAgainAtFar(double delayBeforeShot) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                /*new ParallelGroup(
                        //new FollowPath(toShootAtFarFromLastPose),
                        intake.railDownAuto
                ),*/
                new Delay(delayBeforeShot),
                new ParallelGroup(
                        intake.shootAllThree
                )
        ));
        lastPose = farShootingPose;
    }

    protected void intake1(double delayAfterIntake) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new InstantCommand(() -> FIREWHEELS_ON=false),
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake1FromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake1)
                ),
                new Delay(delayAfterIntake),
                intake.stopIntake
        ));
        lastPose = intake1EndPose;
    }

    protected void intake2(double delayAfterIntake) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new InstantCommand(() -> FIREWHEELS_ON=false),
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake2FromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake2)
                ),
                new Delay(delayAfterIntake),
                intake.stopIntake
        ));
        lastPose = intake2EndPose;
    }

    protected void intake3(double delayAfterIntake) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new InstantCommand(() -> FIREWHEELS_ON=false),
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake3FromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake3)
                ),
                new Delay(delayAfterIntake),
                intake.stopIntake
        ));
        lastPose = intake3EndPose;
    }

    protected void intakeHP(double delayAfterIntake) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new InstantCommand(() -> FIREWHEELS_ON=false),
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntakeHPFromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intakeHP)
                ),
                new Delay(delayAfterIntake),
                intake.stopIntake
        ));
        lastPose = intakeHPEndPose;
    }

    protected void openGate(double delayAfterOpenGate) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(
                intake.stopIntake,
                new FollowPath(openGate),
                new Delay(delayAfterOpenGate)
        );
    }

    protected void parkAtFront() {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new ParallelGroup(
                new InstantCommand(() -> FIREWHEELS_ON=false),
                intake.firewheelsOff,
                new InstantCommand(() -> HOOD_POS = 0),
                new InstantCommand(() -> turret.setFixedAngleCustom(0)),
                intake.firewheelsOff,
                flywheel.stopFlywheel,
                new FollowPath(parkAtFrontFromLastPose)
        ));
    }

    protected void parkAtBack() {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new ParallelGroup(
                new InstantCommand(() -> FIREWHEELS_ON=false),
                intake.firewheelsOff,
                new InstantCommand(() -> HOOD_POS = 0),
                new InstantCommand(() -> turret.setFixedAngleCustom(0)),
                intake.firewheelsOff,
                flywheel.stopFlywheel,
                new FollowPath(parkAtBackFromLastPose)
        ));
    }

    protected void parkAtCustomPose(Pose park) {
        customParkPose = true;
        AutoPaths.setFrontParkPose(park);
        AutoPaths.setParkAngle(park.getHeading());
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new ParallelGroup(
                new InstantCommand(() -> turret.setTurretAngle(0)),
                intake.firewheelsOff,
                flywheel.stopFlywheel,
                new FollowPath(parkAtFrontFromLastPose)
        ));
    }
}