package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretLights;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import org.firstinspires.ftc.teamcode.utilities.Motif;
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
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.pedropathing.follower.Follower;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.util.ElapsedTime;

import dev.nextftc.ftc.NextFTCOpMode;

public abstract class AutoTemplate extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                turret = new Turret()
        );
    }
    protected TurretLights turretLights;
    protected CommandGroup autonomousCommands;
    protected Alliance alliance = null; // default value
    protected Pose startPose;
    public static Pose lastPose; // Tracker for Pose for path generation
    ElapsedTime initTimer = new ElapsedTime(), loopTime = new ElapsedTime();
    double timeToInit = 0;
    Turret turret;
    Flywheel flywheel;
    Intake intake;
    TelemetryManager telemetryM;
    Follower follower;
    double HOOD_POS, secondsBeforeIntakeOff = 0.5, maxLoopTimeMS = 0;
    boolean FIREWHEELS_ON = false, hasResetEncoders = false;
    public boolean isDone;
    public static boolean reverseIntake = false;



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

        // PedroComponent is what NextFTC uses, we want to use the same follower
        // and not accidentally create 2 followers
        follower = PedroComponent.follower();

        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        // default alliance color
        alliance = Alliance.BLUE;
        initAuto();

        AutoPaths.generatePoses(follower);

        follower.setStartingPose(startPose);
        follower.update();

        turretLights = new TurretLights(hardwareMap, telemetry);


        if (alliance == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
        }

        hasResetEncoders = false;
        timeToInit = initTimer.seconds();
    }

    @Override
    public void onWaitForStart() {
        if (!hasResetEncoders) {
            if (intake.getMotif().equals(Motif.gPP)) {
                turretLights.gPP();
            } else if (intake.getMotif().equals(Motif.pGP)) {
                turretLights.pGP();
            } else if (intake.getMotif().equals(Motif.pPG)) {
                turretLights.pPG();
            }
            flywheel.resetHoodEncoder();
            turret.zeroTurret();
            intake.resetAgitatorEncoder();
            hasResetEncoders = true;
        } else {
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
    }

    @Override
    public void onStartButtonPressed() {
        autonomousCommands.schedule();
        System.gc(); // Garbage Collect
    }
    @Override
    public void onUpdate() {
        loopTime.reset();
        turret.setTurretStateFixed();
        turret.update();
        follower.update();

        flywheel.setHoodGoalPos(HOOD_POS);

        Drawing.drawOnlyCurrent(follower);

        if (FIREWHEELS_ON) {
            intake.runFireWheels();
        }

        flywheel.update();
        intake.update();

        if(loopTime.milliseconds() > maxLoopTimeMS) {
            maxLoopTimeMS = loopTime.milliseconds();
        }
        telemetry.addLine("---- AUTO ----");
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("looptime", loopTime.milliseconds());
        telemetry.addData("max looptime", maxLoopTimeMS);
        telemetry.addData("pose", follower.isBusy() ? follower.getCurrentPath().getPose(1) : "mewo");
        telemetry.update();
    }

    @Override
    public void onStop() {
        // Workaround for a Pedro bug that makes the follower read (0,0) on stop
        if (follower.getPose().getX() != 0) {
            OpModeTransfer.currentPose = follower.getPose();
        }
        OpModeTransfer.alliance = alliance;
        OpModeTransfer.hasBeenTransferred = true;
    }

    protected Command runFirewheels = new InstantCommand(() -> FIREWHEELS_ON=true);

    protected void setTurretFixedClose() {
        autonomousCommands = autonomousCommands.then(
                new InstantCommand(() -> setTurretFixed(alliance, true))
        );
    }

    protected void setTurretFixedFar() {
        autonomousCommands = autonomousCommands.then(
                new InstantCommand(() -> setTurretFixed(alliance, false))
        );
    }

    /**
     * Sets the turret at a fixed angle for auto, based on position and alliance
     */
    private void setTurretFixed(Alliance alliance, boolean isClose) {
        turret.setAlliance(alliance);
        turret.setFixedAngle(alliance, isClose);
    }

    protected void setTurretAuto() {
        autonomousCommands = autonomousCommands.then(
                new InstantCommand(() -> turret.setTurretStateAuto())
        );
    }

    protected void setHoodPosClose() {
        autonomousCommands = autonomousCommands.then(new InstantCommand(() -> setHoodPos(true)));
    }

    protected void setHoodPosFar() {
        autonomousCommands = autonomousCommands.then(new InstantCommand(() -> setHoodPos(false)));
    }

    protected void setHoodPos(boolean isClose) {
        if (isClose) {
            HOOD_POS = Hood.HOOD_AUTON_CLOSE_POS;
        } else {
            HOOD_POS = Hood.HOOD_AUTON_FAR_POS;
        }
    }

    protected void startAsBlue() {
        alliance = Alliance.BLUE;
        AutoPaths.alliance = alliance;
    }

    protected void startAsRed() {
        alliance = Alliance.RED;
        AutoPaths.alliance = alliance;
    }

    protected void startAtBack() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(79, 9.5, Math.toRadians(0));
        } else {
            startPose = new Pose(63.25, 9.5,Math.toRadians(180));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
        AutoPaths.generatePoses(follower);
        setHoodPos(false);
        setTurretFixed(alliance, false);
    }

    protected void startAtFront() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(110, 134.25, Math.toRadians(-94.95));
        } else {
            startPose = new Pose(34, 134.25,Math.toRadians(-85.05));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
        AutoPaths.generatePoses(follower);
        setHoodPos(true);
        setTurretFixed(alliance, true);
    }

    protected void startAtCustomPose(Pose pose) {
        startPose = pose;
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
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
        toShootAtCloseFromLastPose = generatePath(AutoTemplate.lastPose, closeShootingPose);
        shootCloseJiggle = generatePathShortCallback(closeShootingPose, closeShootingPoseJiggle);
        generateShootCommandWithJiggle(toShootAtCloseFromLastPose, closeShootingPose, shootCloseJiggle, delayBeforeShot);
    }

    protected void shootAllThreeAtCloseCurved(double delayBeforeShot) {
        toShootAtCloseFromLastPoseCurved = generatePathCurve(AutoTemplate.lastPose, curveIntake2, closeShootingPose);
        shootCloseJiggle = generatePathShortCallback(closeShootingPose, closeShootingPoseJiggle);
        generateShootCommandWithJiggle(toShootAtCloseFromLastPoseCurved, closeShootingPose, shootCloseJiggle, delayBeforeShot);
    }

    protected void shootAllThreeAtFar(double delayBeforeShot) {
        toShootAtFarFromLastPose = generatePathWithVelocityConstraint(AutoTemplate.lastPose, farShootingPose, 0.7);
        shootFarJiggle = generatePathShortCallback(farShootingPose, farShootingPoseJiggle);
        generateShootCommandWithJiggle(toShootAtFarFromLastPose, farShootingPose, shootFarJiggle, delayBeforeShot);
    }

    /**
     * Generates a normal shoot command
     * do NOT use on blue paths, pedro bugs out. instead, use generateShootCommandWithJiggle()
     * @param toShootPath Path to follow thatgoes to shoot
     * @param endPose Shooting pose
     * @param delayBeforeShot pass through for delay
     */
    private void generateShootCommand(PathChain toShootPath, Pose endPose, double delayBeforeShot) {
        shootCloseJiggle = generatePathShortCallback(toShootPath.endPose(), closeShootingPoseJiggle);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new FollowPath(toShootPath, false),
                        new Delay(secondsBeforeIntakeOff).then(
                                intake.stopIntakeNoReverse,
                                intake.railDownAuto
                        ),
                        runFirewheels
                ),
                new Delay(delayBeforeShot),
                new ParallelGroup(
                        new InstantCommand(() -> intake.turnIsShootingTrue()),
                        intake.shootAllThree()
                )
        ));
        lastPose = endPose;
    }

    /**
     * "Jiggle" is our work-around to a Pedro bug.
     * It is a short path which simulates holding the point.
     * @param toShootPath Path to follow thatgoes to shoot
     * @param endPose Shooting pose
     * @param jigglePath the Jiggle path
     * @param delayBeforeShot pass through for delay
     */
    private void generateShootCommandWithJiggle(PathChain toShootPath, Pose endPose, PathChain jigglePath, double delayBeforeShot) {
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        flywheel.waitUntilFlywheelAtVel(2),
                        new FollowPath(toShootPath, false),
                        stopIntake(),
                        runFirewheels
                ),
                new ParallelGroup(
                        intake.railDownAuto,
                        new Delay(delayBeforeShot)
                ),
                new ParallelGroup(
                        followJigglePath(jigglePath),
                        new InstantCommand(() -> intake.turnIsShootingTrue()),
                        intake.shootAllThree()
                )
        ));
        lastPose = endPose;
    }

    private Command stopIntake() {
        if (reverseIntake) {
            reverseIntake = false;
            return intake.stopIntake;
        }
        return intake.stopIntakeNoReverse;
    }

    /**
     * Custom Command to follow a jiggle path.
     * We need to use a custom Command in order to end purely based on time.
     * @param path to follow
     * @return Command
     */
    private Command followJigglePath(PathChain path) {
        ElapsedTime jiggleTimer = new ElapsedTime();
        return new LambdaCommand()
                .setStart(
                        () -> {
                            jiggleTimer.reset();
                            // TODO: MAY NEED TO CHANGE TO PedroComponent.follower().followPath
                            follower.followPath(path,false);
                        }
                )
                .setStop(interrupted -> follower.breakFollowing())
                .setIsDone(() -> jiggleTimer.milliseconds() > 700); //500
    }

    protected void intake1(double delayAfterIntake) {
        lineUpForIntake1FromLastPose = generatePath(AutoTemplate.lastPose, intake1StartPose);
        intake1 = generatePath(intake1StartPose, intake1EndPose);
        generateIntakeCommand(lineUpForIntake1FromLastPose, intake1, intake1EndPose, delayAfterIntake);
    }

    protected void intake2(double delayAfterIntake) {
        lineUpForIntake2FromLastPose = generatePath(AutoTemplate.lastPose, intake2StartPose);
        intake2 = generatePath(intake2StartPose, intake2EndPose);
        generateIntakeCommand(lineUpForIntake2FromLastPose, intake2, intake2EndPose, delayAfterIntake);
    }

    protected void intake3(double delayAfterIntake) {
        lineUpForIntake3FromLastPose = generatePath(AutoTemplate.lastPose, intake3StartPose);
        intake3 = generatePath(intake3StartPose, intake3EndPose);
        generateIntakeCommand(lineUpForIntake3FromLastPose, intake3, intake3EndPose, delayAfterIntake);
    }

    protected void intakeHP(double delayAfterIntake) {
        lineUpForIntakeHPFromLastPose = generatePath(AutoTemplate.lastPose, intakeHPStartPose);
        intakeHP = generatePath(intakeHPStartPose, intakeHPEndPose);
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new InstantCommand(() -> reverseIntake = true),
                        new InstantCommand(() -> FIREWHEELS_ON=false),
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntakeHPFromLastPose),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intakeHP)
                ),
                new Delay(delayAfterIntake)
        ));
        lastPose = intakeHPEndPose;
    }

    /**
     * Shorthand method to easily generate new intake Commands.
     * @param lineUpForIntake PathChain that goes to the beginning of intake
     * @param intakePath PathChain that actually intakes
     * @param endPose Pose that the path ends on
     * @param delayAfterIntake Pass through a delay after intake
     */
    private void generateIntakeCommand(PathChain lineUpForIntake, PathChain intakePath, Pose endPose, double delayAfterIntake) {
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new InstantCommand(() -> FIREWHEELS_ON=false),
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intakePath)
                ),
                new Delay(delayAfterIntake)
                //intake.stopIntakeNoReverse
        ));
        lastPose = endPose;
    }

    /**
     * Shorthand method to easily generate new intake Commands.
     * @param lineUpForIntake PathChain that goes to the beginning of intake
     * @param intakePath PathChain that actually intakes
     * @param endPose Pose that the path ends on
     * @param delayAfterIntake Pass through a delay after intake
     */
    private void generateIntakeCommandReverse(PathChain lineUpForIntake, PathChain intakePath, Pose endPose, double delayAfterIntake) {
        autonomousCommands = autonomousCommands.then(new SequentialGroup(
                new ParallelGroup(
                        new InstantCommand(() -> reverseIntake = true),
                        new InstantCommand(() -> FIREWHEELS_ON=false),
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intakePath)
                ),
                new Delay(delayAfterIntake)
        ));
        lastPose = endPose;
    }

    protected void waitUntilFlywheelAtSpeed(double thresholdTimeSec) {
        autonomousCommands = autonomousCommands.then(
                flywheel.waitUntilFlywheelAtVel(thresholdTimeSec)
        );
    }

    protected void openGate(double delayAfterOpenGate) {
        lineUpForOpenGateFromLastPose = generatePath(AutoTemplate.lastPose, openGateStartPose);
        openGate = generatePath(openGateStartPose, openGateEndPose);
        autonomousCommands = autonomousCommands.then(
                intake.stopIntakeNoReverse,
                new FollowPath(openGate),
                new Delay(delayAfterOpenGate)
        );
    }

    protected void parkAtFront() {
        customParkPose = false;
        AutoPaths.generatePoses(follower);
        parkAtFrontFromLastPose = generatePath(AutoTemplate.lastPose, frontParkPose);
        generateParkCommand(parkAtFrontFromLastPose);
    }

    protected void parkAtBack() {
        customParkPose = false;
        AutoPaths.generatePoses(follower);
        parkAtBackFromLastPose = generatePath(AutoTemplate.lastPose, backParkPose);
        generateParkCommand(parkAtBackFromLastPose);
    }

    protected void parkAtCustomPose(Pose park) {
        customParkPose = true;
        AutoPaths.setFrontParkPose(park);
        AutoPaths.setParkAngle(park.getHeading());
        AutoPaths.generatePoses(follower);
        generateParkCommand(parkAtFrontFromLastPose);
    }

    /**
     * Generates a park Command, which turns everything off and follows the path.
     * @param parkPath path to follow
     */
    private void generateParkCommand(PathChain parkPath) {
        autonomousCommands = autonomousCommands.then(
                new ParallelGroup(
                    new InstantCommand(() -> FIREWHEELS_ON=false),
                    intake.firewheelsOff,
                    new InstantCommand(() -> HOOD_POS = 0),
                    new InstantCommand(() -> turret.setFixedAngleCustom(0)),
                    flywheel.stopFlywheel,
                    new FollowPath(parkPath)
                )
        );
    }



}