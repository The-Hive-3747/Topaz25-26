package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretLights;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import org.firstinspires.ftc.teamcode.utilities.Light;
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

import dev.nextftc.ftc.NextFTCOpMode;

public abstract class AutoTemplate extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                aimbot = new Aimbot(),
                turret = new Turret(),
                light = new Light()
        );
    }
    protected CommandGroup autonomousCommands;
    protected Alliance alliance = Alliance.BLUE; // default value
    protected Pose startPose, parkPose;
    protected double parkAngle;
    Light light;
    Aimbot aimbot;
    Turret turret;
    Flywheel flywheel;
    TurretLights turretLights;
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
        initAuto();

        follower = Constants.createFollower(hardwareMap);

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        AutoPaths.alliance = alliance;
        if (startPose != null) {
            AutoPaths.setStartPose(startPose);
        }
        if (parkPose != null) {
            AutoPaths.setParkPose(parkPose);
        }
        if (parkAngle != 0.0d) {
            AutoPaths.setParkAngle(parkAngle);
        }
        AutoPaths.generatePaths(follower);

        follower.setStartingPose(startPose);
        follower.update();

        turret.setAlliance(alliance);
        turret.setFixedAngle(alliance);
        aimbot.setAlliance(alliance);

        turretLights = new TurretLights(hardwareMap, telemetry);

        // TODO: ADD TURRET LIGHTS
        /*
        if (alliance == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
        } */

        if (autonomousCommands == null) {
            autonomousCommands = new SequentialGroup(
                    new ParallelGroup(
                            startAimbotFlywheel,
                            new FollowPath(toShootFromStart),
                            intake.railDownAuto
                    ),
                    new Delay(1.5),
                    new ParallelGroup(
                            intake.shootAllThree
                    ),
                    new ParallelGroup(
                            intake.firewheelsOff,
                            new FollowPath(lineUpForIntake1),
                            intake.startIntake
                    ),
                    new ParallelGroup(
                            intake.startIntake,
                            new FollowPath(intake1)
                    ),
                    new Delay(1),
                    new FollowPath(toShootFromIntake1),
                    intake.stopIntake,
                    new ParallelGroup(
                            intake.shootAllThree
                    ),
                    new ParallelGroup(
                            intake.firewheelsOff,
                            new FollowPath(lineUpForIntake2),
                            intake.startIntake
                    ),
                    new FollowPath(intake2),
                    new Delay(1),
                    intake.stopIntake,
                    new FollowPath(toShootFromIntake2),
                    new Delay(0.3),
                    new ParallelGroup(
                            intake.shootAllThree
                    ),
                    new ParallelGroup(
                            intake.firewheelsOff,
                            new FollowPath(lineUpForIntake3),
                            intake.startIntake
                    ),
                    new FollowPath(intake3),
                    new Delay(1.3),
                    intake.stopIntake,
                    new FollowPath(toShootFromIntake3),
                    new Delay(0.3),
                    new ParallelGroup(
                            intake.shootAllThree
                    ),
                    new ParallelGroup(
                            intake.firewheelsOff,
                            flywheel.stopFlywheel,
                            new FollowPath(park)
                    )
            );
        }

        turret.zeroTurret();
    }

    @Override
    public void onWaitForStart() {
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("alliance", alliance);
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        flywheel.resetHoodEncoder();
        autonomousCommands.schedule();
    }
    @Override
    public void onUpdate() {
        //turret.setCurrentPose(PedroComponent.follower().getPose(), PedroComponent.follower().getVelocity());
        turret.update();
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


    //TODO: UPDATE FOR TOPAZ
    /**
     * Stops everything at the end of auto
     * @return CommandGroup
     */
    protected CommandGroup stopIntakeFlywheelAndTurret() {
        return new ParallelGroup(
                //new InstantCommand(
                //       () -> flywheel.setHoodGoalPos(0)
                //),
                turret.setTurretForward,
                flywheel.stopFlywheel,
                intake.stopIntake,
                intake.stopTransfer
        );
    }

    // TODO: UPDATE FOR TOPAZ
    /**
     * Starts flywheel & turret at the beginning of auto
     * @return CommandGroup
     */
    protected CommandGroup startIntakeFlywheelAndTurret() {
        return new ParallelGroup(
                intake.fastIntake,
                //this.startAimbotFlywheel,
                turret.setTurretFixed
        );

    }
}