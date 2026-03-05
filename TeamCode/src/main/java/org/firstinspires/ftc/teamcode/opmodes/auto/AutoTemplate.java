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
                //aimbot = new Aimbot(),
                turret = new Turret()
                //light = new Light()
        );
    }
    protected CommandGroup autonomousCommands;
    protected Alliance alliance = Alliance.BLUE; // default value
    protected Pose startPose, parkPose;
    ElapsedTime looptimer = new ElapsedTime();
    public static Pose lastPose;
    protected double parkAngle;
    Turret turret;
    Flywheel flywheel;
    Intake intake;
    TelemetryManager telemetryM;
    Follower follower;
    double FLYWHEEL_VEL, HOOD_POS;
    boolean FLYWHEEL_ON = false;


    /**
     * An abstract method to be used by child classes in order to init stuff in auto.
     * Should override alliance, autonomousCommands, and startPose
     */
    public abstract void initAuto();


    @Override
    public void onInit() {
        autonomousCommands = new SequentialGroup(
                new InstantCommand(() -> telemetry.addLine())
        );

        follower = Constants.createFollower(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        if (parkPose != null) {
            AutoPaths.setParkPose(parkPose);
        }
        if (parkAngle != 0.0d) {
            AutoPaths.setParkAngle(parkAngle);
        }

        initAuto();

        AutoPaths.generatePaths(follower);

        follower.setStartingPose(startPose);
        follower.update();

        //turret.setAlliance(alliance);
        //turret.setFixedAngle(alliance);

        //turretLights = new TurretLights(hardwareMap, telemetry);

        // TODO: ADD TURRET LIGHTS
        /*
        if (alliance == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
        } */

        //turret.zeroTurret();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("alliance", alliance);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        autonomousCommands.schedule();
        flywheel.resetHoodEncoder();
    }
    @Override
    public void onUpdate() {
        //turret.setCurrentPose(PedroComponent.follower().getPose(), PedroComponent.follower().getVelocity());
        //turret.update();
        follower.update();

        //aimbot.setCurrentPose(follower.getPose(), follower.getVelocity());
        //aimbot.update();
        if (FLYWHEEL_ON) {
            FLYWHEEL_VEL = Flywheel.AUTON_SHOOT_VEL; //aimbot.getAimbotValues().velocity;
        } else {
            FLYWHEEL_VEL = 0;
        }
        HOOD_POS = Hood.AUTON_HOOD_POS; //aimbot.getAimbotValues().hoodPos;
        //flywheel.setHoodGoalPos(HOOD_POS);
        flywheel.setTargetVel(FLYWHEEL_VEL);
        //flywheel.setTargetVel(FLYWHEEL_VEL);
        //flywheel.setTargetVel(0);

        Drawing.drawOnlyCurrent(follower);

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

    public Command startAimbotFlywheel = new InstantCommand(
            () -> FLYWHEEL_ON = true
    );


    protected void startAsBlue() {
        alliance = Alliance.BLUE;
        AutoPaths.alliance = alliance;
        turret.setAlliance(alliance);
        turret.setFixedAngle(alliance);
    }

    protected void startAsRed() {
        alliance = Alliance.RED;
        AutoPaths.alliance = alliance;
        turret.setAlliance(alliance);
        turret.setFixedAngle(alliance);
    }

    protected void startAtBack() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(80.75, 7.085,Math.toRadians(90));
        } else {
            startPose = new Pose(63.25, 7.585,Math.toRadians(90));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void startAtFront() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(109.5, 135.8, Math.toRadians(-94.95));
        } else {
            startPose = new Pose(34.5, 135.8,Math.toRadians(-85.05));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void startAtCustomPose(Pose pose) {
        startPose = pose;
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void turnFlywheelOn() {
        autonomousCommands = autonomousCommands.then(startAimbotFlywheel);
    }

    protected void shootAllThreeAtFront(double delayBeforeShot) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
            new ParallelGroup(
                startAimbotFlywheel,
                new FollowPath(toShootAtFrontFromLastPose),
                intake.railDownAuto
            ),
                new Delay(delayBeforeShot),
                new ParallelGroup(
                        intake.shootAllThree
                )
        ));
        lastPose = frontShootingPose;
    }

    protected void intake1(double delayAfterIntake) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake1FromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake1)
                ),
                new Delay(delayAfterIntake)
        ));
        lastPose = intake1EndPose;
    }

    protected void intake2(double delayAfterIntake) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake2FromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake1)
                ),
                new Delay(delayAfterIntake)
        ));
        lastPose = intake2EndPose;
    }

    protected void intake3(double delayAfterIntake) {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake3FromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake1)
                ),
                new Delay(delayAfterIntake)
        ));
        lastPose = intake3EndPose;
    }

    protected void parkAtFront() {
        AutoPaths.generatePaths(follower);
        autonomousCommands = autonomousCommands.then(new ParallelGroup(
                intake.firewheelsOff,
                flywheel.stopFlywheel,
                new FollowPath(parkAtFrontFromLastPose)
        ));
    }
}