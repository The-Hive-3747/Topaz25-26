package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.HashSet;
import java.util.Objects;

import dev.nextftc.ftc.NextFTCOpMode;

public abstract class AutoTemplate extends NextFTCOpMode {
    {
        addComponents(
                flywheel = new Flywheel(),
                intake = new Intake(),
                turret = new Turret()
        );
    }

    protected Alliance alliance = Alliance.BLUE;
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
    double flywheelVel = 0;
    boolean FIREWHEELS_ON = false;

    // Path building
    protected PathBuilder pathBuilder;
    protected PathChain allPaths;
    protected int pathIndex = 0;


    protected HashMap<Integer, Runnable> pauseActions = new HashMap<>();
    protected HashMap<Integer, Runnable> entryActions = new HashMap<>();
    protected HashSet<Integer> firedEntryActions = new HashSet<>();
    protected HashSet<Integer> firedPauseActions = new HashSet<>();

    protected ElapsedTime shootPauseTimer = null;
    protected boolean waitingToResume = false;
    protected double shootPauseDuration = 1.25; // tune this
    protected boolean hasShot = false;
    protected double settleTime = 0.5;
    protected Runnable pendingShootAction = null;
    protected int lastSeenChainIndex = -1;

    /**
     * Child classes override this to define alliance, start pose,
     * and call the modular path-building methods.
     */
    public abstract void initAuto();

    @Override
    public void onInit() {
        customParkPose = false;
        initTimer.reset();

        follower = Constants.createFollower(hardwareMap);
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();

        initAuto();

        allPaths = pathBuilder.build();

        follower.setStartingPose(startPose);
        follower.update();

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
        follower.followPath(allPaths);
        lastSeenChainIndex = -1;
    }

    @Override
    public void onUpdate() {
        int currentIndex = follower.getChainIndex();

        telemetry.addLine("----- AUTO TELEMETRY -----");
        telemetry.addData("index", pathIndex);
        telemetry.addData("pause?", waitingToResume);
        telemetry.addData("pauseActions", pauseActions);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("chain index", currentIndex);
        telemetry.addData("lastSeenChainIndex", lastSeenChainIndex);
        telemetry.addData("waiting to resume", waitingToResume);
        telemetry.addData("firedPauseActions", firedPauseActions);
        telemetry.addData("isBusy", follower.isBusy());

        if (currentIndex != lastSeenChainIndex && !waitingToResume) {
            if (pauseActions.containsKey(currentIndex) && !firedPauseActions.contains(currentIndex)) {
                firedPauseActions.add(currentIndex);
                follower.pausePathFollowing();
                pendingShootAction = pauseActions.get(currentIndex);
                hasShot = false;
                shootPauseTimer = new ElapsedTime();
                waitingToResume = true;
            } else {
                if (entryActions.containsKey(currentIndex) && !firedEntryActions.contains(currentIndex)) {
                    Objects.requireNonNull(entryActions.get(currentIndex)).run();
                    firedEntryActions.add(currentIndex);
                }
                lastSeenChainIndex = currentIndex;
            }
        }

        if (waitingToResume && !hasShot && shootPauseTimer != null && shootPauseTimer.seconds() > settleTime) {
            hasShot = true;
            Objects.requireNonNull(pendingShootAction).run();
        }

        if (waitingToResume && shootPauseTimer != null
                && shootPauseTimer.seconds() > shootPauseDuration) {
            waitingToResume = false;
            follower.resumePathFollowing();
            pendingShootAction = null;

            if (entryActions.containsKey(currentIndex)
                    && !firedEntryActions.contains(currentIndex)) {
                Objects.requireNonNull(entryActions.get(currentIndex)).run();
                firedEntryActions.add(currentIndex);
            }
            lastSeenChainIndex = currentIndex;
        }

        turret.setTurretStateFixed();
        turret.update();
        follower.update();

        flywheel.setHoodGoalPos(HOOD_POS);
        flywheel.setTargetVel(flywheelVel);

        Drawing.drawOnlyCurrent(follower);

        if (FIREWHEELS_ON) {
            intake.runFireWheels();
        }

        flywheel.update();
        intake.update();
        telemetry.update();
    }

    @Override
    public void onStop() {
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
            startPose = new Pose(63.25, 9.5, Math.toRadians(180));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void startAtFront() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(110, 134.25, Math.toRadians(-94.95));
        } else {
            startPose = new Pose(34, 135.5, Math.toRadians(-85.05));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void startAtCustomPose(Pose pose) {
        startPose = pose;
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    /**
     * Call in initAuto() BEFORE any path-building methods.
     * Sets up poses and initializes the path builder.
     */
    protected void beginPathBuilding() {
        AutoPaths.generatePoses(follower);
        pathBuilder = follower.pathBuilder();
        pathIndex = 0;
        pauseActions.clear();
        entryActions.clear();
        firedEntryActions.clear();
        firedPauseActions.clear();
    }

    protected void turnFlywheelOnForFront() {
        flywheelVel = Flywheel.FLYWHEEL_AUTO_TARGET_VEL_FRONT;
    }

    protected void turnFlywheelOnForBack() {
        flywheelVel = Flywheel.FLYWHEEL_AUTO_TARGET_VEL_BACK;
    }

    protected void turnFlywheelOnCustom(double power) {
        flywheelVel = power;
    }

    protected void intake1() {
        entryActions.put(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intake1StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake1StartPose.getHeading());
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intake1StartPose, intake1EndPose))
                .setLinearHeadingInterpolation(intake1StartPose.getHeading(), intake1EndPose.getHeading());
        pathIndex++;

        lastPose = intake1EndPose;
    }

    protected void intake2() {
        entryActions.put(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intake2StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake2StartPose.getHeading());
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intake2StartPose, intake2EndPose))
                .setLinearHeadingInterpolation(intake2StartPose.getHeading(), intake2EndPose.getHeading());
        pathIndex++;

        lastPose = intake2EndPose;
    }

    protected void intake3() {
        entryActions.put(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intake3StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake3StartPose.getHeading());
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intake3StartPose, intake3EndPose))
                .setLinearHeadingInterpolation(intake3StartPose.getHeading(), intake3EndPose.getHeading());
        pathIndex++;

        lastPose = intake3EndPose;
    }

    protected void intakeHP() {
        entryActions.put(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intakeHPStartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intakeHPStartPose.getHeading());
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intakeHPStartPose, intakeHPEndPose))
                .setLinearHeadingInterpolation(intakeHPStartPose.getHeading(), intakeHPEndPose.getHeading());
        pathIndex++;

        lastPose = intakeHPEndPose;
    }

    protected void openGate() {
        entryActions.put(pathIndex, () -> intake.stopIntake());
        pathBuilder
                .addPath(new BezierLine(lastPose, openGateStartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), openGateStartPose.getHeading());
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(openGateStartPose, openGateEndPose))
                .setLinearHeadingInterpolation(openGateStartPose.getHeading(), openGateEndPose.getHeading());
        pathIndex++;

        lastPose = openGateEndPose;
    }

    /**
     * Appends a path to the close shooting pose and registers a pause
     * on the NEXT path index. When Pedro transitions past this path,
     * onUpdate() catches it and pauses before follower.update() runs.
     * Index layout example:
     *   pathIndex N:   drive to closeShootingPose (entry action preps turret/hood)
     *   pathIndex N+1: whatever comes next (pause registered here, shoot action runs)
     */
    protected void shootAtClose() {
        entryActions.put(pathIndex, () -> {
            intake.stopIntake();
            setTurretFixedClose();
            setHoodPosClose();
            intake.railDown();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading())
                .setVelocityConstraint(0.8);
        pathIndex++;

        // Pause fires when Pedro transitions to the next path (pathIndex is
        // now pointing at whatever path comes after this one in the child auto)
        pauseActions.put(pathIndex, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        lastPose = closeShootingPose;
    }

    protected void shootAtCloseCurved() {
        entryActions.put(pathIndex, () -> {
            intake.stopIntake();
            setTurretFixedClose();
            setHoodPosClose();
            intake.railDown();
        });
        pathBuilder
                .addPath(new BezierCurve(lastPose, curveIntake2, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading());
        pathIndex++;

        pauseActions.put(pathIndex, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        lastPose = closeShootingPose;
    }

    protected void shootAtFar() {
        entryActions.put(pathIndex, () -> {
            intake.stopIntake();
            setTurretFixedFar();
            setHoodPosFar();
            intake.railDown();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, farShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), farShootingPose.getHeading())
                .setVelocityConstraint(0.7);
        pathIndex++;

        pauseActions.put(pathIndex, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        lastPose = farShootingPose;
    }

    protected void parkAtFront() {
        entryActions.put(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            HOOD_POS = 0;
            flywheelVel = 0;
            turret.setFixedAngleCustom(0);
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, frontParkPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), frontParkPose.getHeading());
        pathIndex++;

        lastPose = frontParkPose;
    }

    protected void parkAtBack() {
        entryActions.put(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            HOOD_POS = 0;
            flywheelVel = 0;
            turret.setFixedAngleCustom(0);
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, backParkPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), backParkPose.getHeading());
        pathIndex++;

        lastPose = backParkPose;
    }

    protected void parkAtCustomPose(Pose park) {
        customParkPose = true;
        AutoPaths.setFrontParkPose(park);
        AutoPaths.setParkAngle(park.getHeading());

        entryActions.put(pathIndex, () -> {
            turret.setTurretAngle(0);
            intake.firewheelsOff();
            flywheelVel = 0;
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, park))
                .setLinearHeadingInterpolation(lastPose.getHeading(), park.getHeading());
        pathIndex++;

        lastPose = park;
    }
}