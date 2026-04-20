package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

import com.bylazar.camerastream.PanelsCameraStream;
import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.robotcore.external.stream.CameraStreamSource;
import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.utilities.AimbotValues;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Artifact;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import org.firstinspires.ftc.teamcode.utilities.Motif;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.vision.limelight.LimelightComponent;

import com.pedropathing.follower.Follower;
import com.qualcomm.robotcore.util.ElapsedTime;

import java.util.HashMap;
import java.util.HashSet;
import java.util.List;
import java.util.Objects;

import dev.nextftc.ftc.NextFTCOpMode;

public abstract class AutoTemplate extends NextFTCOpMode {
    {
        addComponents(
                flywheel = new Flywheel(),
                intake = new Intake(),
                turret = new Turret(),
                limelight = new LimelightComponent()
        );
    }

    protected Alliance alliance = Alliance.BLUE;
    protected Pose startPose;
    public static Pose lastPose;

    Turret turret;
    Flywheel flywheel;
    LimelightComponent limelight;
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
    protected double toIntakePower = 0.5;
    protected double toIntakeBrakeTValue = 0.7;
    protected double intakePower = 1;
    protected double shootBrakeDistance = 18;
    protected double shootBrakePower = 0.5;

    // Pause/resume
    protected HashMap<Integer, Runnable> pauseActions = new HashMap<>();
    protected HashMap<Integer, Runnable> entryActions = new HashMap<>();
    protected HashSet<Integer> firedEntryActions = new HashSet<>();
    protected HashSet<Integer> firedPauseActions = new HashSet<>();
    protected HashMap<Integer, Double> pauseDurations = new HashMap<>();
    protected ElapsedTime shootPauseTimer = null;
    protected boolean waitingToResume = false;
    protected boolean hasShot = false;
    protected double shootPauseDuration = 1.4;
    protected double gatePauseDuration = 1.1;
    protected double settleTime = 0;
    protected double currentPauseDuration = 0;
    protected Runnable pendingShootAction = null;
    protected int lastSeenChainIndex = -1;

    // Sorted shooting
    protected Motif autoMotif = null;
    protected double sortedShootPauseDuration = 3;
    protected double sortedShotWaitTime = 0.2;
    private enum SortedShotState { IDLE, FIRING_FIRST, WAIT_FIRST, FIRING_SECOND, DONE }
    private SortedShotState sortedShotState = SortedShotState.IDLE;
    private List<Motif.Shoot> currentShotOrder = null;
    private ElapsedTime sortedShotStepTimer = null;

    ElapsedTime initTimer = new ElapsedTime();
    double timeToInit = 0;

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

        limelight.init();

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
        telemetry.addData("order", List.of(intake.getLeftArtifact(), intake.getFrontArtifact(), intake.getRightArtifact()));
        telemetry.addData("motif", !(autoMotif == null) ? autoMotif.toString() : null);
        telemetry.addData("motif", limelight.getAprilTagId());
        telemetry.addData("hasTarget", limelight.getAprilTagId());
        telemetry.addData("index", pathIndex);
        telemetry.addData("pause?", waitingToResume);
        telemetry.addData("pauseActions", pauseActions);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("chain index", currentIndex);
        telemetry.addData("lastSeenChainIndex", lastSeenChainIndex);
        telemetry.addData("waiting to resume", waitingToResume);
        telemetry.addData("firedPauseActions", firedPauseActions);
        telemetry.addData("isBusy", follower.isBusy());

        // Detect chain index transitions
        if (currentIndex != lastSeenChainIndex && !waitingToResume) {
            if (pauseActions.containsKey(currentIndex) && !firedPauseActions.contains(currentIndex)) {
                firedPauseActions.add(currentIndex);
                follower.pausePathFollowing();
                pendingShootAction = pauseActions.get(currentIndex);
                currentPauseDuration = pauseDurations.getOrDefault(currentIndex, shootPauseDuration);
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

        // Execute pending shoot action after settle time
        if (waitingToResume && !hasShot && shootPauseTimer != null && shootPauseTimer.seconds() > settleTime) {
            hasShot = true;
            Objects.requireNonNull(pendingShootAction).run();
        }

        // Sorted shot state machine
        if (sortedShotState != SortedShotState.IDLE && sortedShotStepTimer != null) {
            switch (sortedShotState) {
                case FIRING_FIRST:
                    fireForShot(currentShotOrder.get(0));
                    sortedShotStepTimer.reset();
                    sortedShotState = currentShotOrder.size() > 1
                            ? SortedShotState.WAIT_FIRST
                            : SortedShotState.DONE;
                    break;
                case WAIT_FIRST:
                    if (sortedShotStepTimer.seconds() > sortedShotWaitTime) {
                        sortedShotState = SortedShotState.FIRING_SECOND;
                    }
                    break;
                case FIRING_SECOND:
                    fireForShot(currentShotOrder.get(1));
                    sortedShotState = SortedShotState.DONE;
                    break;
                case DONE:
                    sortedShotState = SortedShotState.IDLE;
                    currentShotOrder = null;
                    break;
            }
        }

        // Resume path following after pause duration (and sorted shot complete)
        if (waitingToResume && shootPauseTimer != null
                && shootPauseTimer.seconds() > currentPauseDuration
                && sortedShotState == SortedShotState.IDLE) {
            waitingToResume = false;
            follower.resumePathFollowing();
            follower.setMaxPower(1.0);
            pendingShootAction = null;

            if (entryActions.containsKey(currentIndex)
                    && !firedEntryActions.contains(currentIndex)) {
                Objects.requireNonNull(entryActions.get(currentIndex)).run();
                firedEntryActions.add(currentIndex);
            }
            lastSeenChainIndex = currentIndex;
        }

        // Subsystem updates
        turret.setTurretStateFixed();
        turret.update();
        follower.update();

        flywheel.setHoodGoalPos(HOOD_POS);
        flywheel.setTargetVel(flywheelVel);

        Drawing.drawOnlyCurrent(follower);

        if (FIREWHEELS_ON) {
            intake.runFireWheels();
        }

        limelight.update(follower.getHeading());
        autoMotif = Motif.getMotif(limelight.getAprilTagId());
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

    // =====================================================
    // SETUP (alliance, start pose, turret, hood, flywheel)
    // =====================================================

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

    protected void startAtFront() {
        if (alliance == Alliance.RED) {
            startPose = new Pose(110, 134.25, Math.toRadians(-94.95));
        } else {
            startPose = new Pose(34, 135.5, Math.toRadians(-85.05));
        }
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
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

    protected void startAtCustomPose(Pose pose) {
        startPose = pose;
        AutoPaths.setStartPose(startPose);
        lastPose = startPose;
    }

    protected void setTurretFixedClose() {
        turret.setAlliance(alliance);
        turret.setFixedAngleClose(alliance);
    }

    protected void setTurretFixedLimelight() {
        turret.setFixedAngleCloseLimelight(alliance);
    }

    protected void setTurretFixedFar() {
        turret.setAlliance(alliance);
        turret.setFixedAngleFar(alliance);
    }

    protected void setTurretFixedClosePark() {
        double goalX, goalY;
        if (alliance == Alliance.RED) {
            goalX = 144; goalY = 144;
        } else {
            goalX = 0; goalY = 144;
        }
        double fieldAngle = Math.atan2(
                goalY - closeShootAndParkPose.getY(),
                goalX - closeShootAndParkPose.getX());
        double turretAngle = Math.toDegrees(
                Turret.normalizeAngle(fieldAngle - closeShootAndParkPose.getHeading() + Math.PI));
        turret.setFixedAngleCustom(turretAngle);
    }

    protected void setHoodPosClose() {
        HOOD_POS = Hood.HOOD_AUTON_CLOSE_POS;
    }

    protected void setHoodPosFar() {
        HOOD_POS = Hood.HOOD_AUTON_FAR_POS;
    }

    protected void setHoodPosClosePark() {
        HOOD_POS = getAimValuesForClosePark().hoodPos;
    }

    protected void turnFlywheelOnForFront() {
        flywheelVel = Flywheel.FLYWHEEL_AUTO_TARGET_VEL_FRONT;
    }

    protected void turnFlywheelOnForBack() {
        flywheelVel = Flywheel.FLYWHEEL_AUTO_TARGET_VEL_BACK;
    }

    protected void turnFlywheelOnForClosePark() {
        flywheelVel = getAimValuesForClosePark().velocity;
    }

    protected void turnFlywheelOnCustom(double power) {
        flywheelVel = power;
    }

    private AimbotValues getAimValuesForClosePark() {
        double aGoalX, aGoalY;
        if (alliance == Alliance.RED) {
            aGoalX = 120; aGoalY = 129;
        } else {
            aGoalX = 20; aGoalY = 129;
        }
        double dist = Math.sqrt(
                Math.pow(aGoalX - closeShootAndParkPose.getX(), 2)
              + Math.pow(aGoalY - closeShootAndParkPose.getY(), 2));

        Aimbot aimbot = new Aimbot();
        aimbot.setAlliance(alliance);
        return aimbot.getAimValues(dist);
    }

    // =====================================================
    // PATH BUILDING INFRASTRUCTURE
    // =====================================================

    protected void beginPathBuilding() {
        AutoPaths.generatePoses(follower);
        pathBuilder = follower.pathBuilder();
        pathBuilder.setGlobalDeceleration();
        pathIndex = 0;
        pauseActions.clear();
        pauseDurations.clear();
        entryActions.clear();
        firedEntryActions.clear();
        firedPauseActions.clear();
    }

    protected void addEntryAction(int index, Runnable action) {
        entryActions.merge(index, action, (a, b) -> () -> {
            a.run();
            b.run();
        });
    }

    private double computeBrakeT(Pose start, Pose end) {
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double pathLength = Math.sqrt(dx * dx + dy * dy);
        return Math.max(0, (pathLength - shootBrakeDistance) / pathLength);
    }

    // =====================================================
    // INTAKE PATHS
    // =====================================================

    private void intakeLine(Pose startPose, Pose endPose, double approachPower, double sweepPower) {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, startPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), startPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(approachPower));
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(startPose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(sweepPower));
        pathIndex++;

        lastPose = endPose;
    }

    private void intakeCurve(Pose startPose, Pose curvePose, Pose endPose, double approachPower, double sweepPower) {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, startPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), startPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(approachPower));
        pathIndex++;

        pathBuilder
                .addPath(new BezierCurve(startPose, curvePose, endPose))
                .setLinearHeadingInterpolation(startPose.getHeading(), endPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(sweepPower));
        pathIndex++;

        lastPose = endPose;
    }

    protected void intake1() {
        intakeLine(intake1StartPose, intake1EndPose, toIntakePower, intakePower);
    }

    protected void intake2() {
        intakeLine(intake2StartPose, intake2EndPose, toIntakePower, intakePower);
    }

    protected void intake3() {
        intakeLine(intake3StartPose, intake3EndPose, toIntakePower, intakePower);
    }

    protected void intakeHP() {
        intakeLine(intakeHPStartPose, intakeHPEndPose, toIntakePower, intakePower);
    }

    protected void intake1Adam() {
        intakeLine(intake1AdamStartPose, intake1AdamEndPose, 0.6, 0.8);
    }

    protected void intake1Swoopspike() {
        intakeCurve(intake1SwoopspikeStartPose, intake1SwoopspikeCurvePose, intake1SwoopspikeEndPose, 0.6, 1);
    }

    protected void intake2Swoopspike() {
        intakeCurve(intake2SwoopspikeStartPose, intake2SwoopspikeCurvePose, intake2SwoopspikeEndPose, 0.6, 1);
    }

    protected void intakeGate() {
        intakeLine(gateIntakeStartPose, gateIntakeEndPose, toIntakePower, 0.5);

        pauseActions.put(pathIndex, () -> {});
        pauseDurations.put(pathIndex, gatePauseDuration);
        addEntryAction(pathIndex, () -> intake.stopIntake());
    }

    protected void openGate() {
        addEntryAction(pathIndex, () -> intake.stopIntakeNoReverse());
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

    // =====================================================
    // SHOOTING PATHS
    // =====================================================

    protected void shootAtClose() {
        double brakeT = computeBrakeT(lastPose, closeShootingPose);
        addEntryAction(pathIndex, this::setHoodPosClose);
        pathBuilder
                .addPath(new BezierLine(lastPose, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading())
                .addParametricCallback(0.3, () -> { intake.stopIntakeNoReverse(); intake.railDown(); FIREWHEELS_ON = true; })
                .addParametricCallback(0, () -> {
                    follower.setMaxPower(1);
                    if (autoMotif == null) {
                        setTurretFixedLimelight();
                    } else {
                        setTurretFixedClose();
                    }
                })
                .addParametricCallback(0.8, this::setTurretFixedClose)
                .addParametricCallback(brakeT, () -> { follower.setMaxPower(0.35); FIREWHEELS_ON = true; });
        pathIndex++;

        pauseActions.put(pathIndex, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        lastPose = closeShootingPose;
    }

    protected void shootAtCloseCurved() {
        double brakeT = computeBrakeT(lastPose, closeShootingPose);
        addEntryAction(pathIndex, () -> {
            setTurretFixedClose();
            setHoodPosClose();
        });
        pathBuilder
                .addPath(new BezierCurve(lastPose, curveIntake2, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading())
                .addParametricCallback(0.3, () -> { intake.stopIntakeNoReverse(); intake.railDown(); FIREWHEELS_ON = true; })
                .addParametricCallback(0, () -> follower.setMaxPower(1))
                .addParametricCallback(brakeT, () -> follower.setMaxPower(0.6));
        pathIndex++;

        pauseActions.put(pathIndex, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        lastPose = closeShootingPose;
    }

    protected void shootAtFar() {
        double brakeT = computeBrakeT(lastPose, farShootingPose);
        addEntryAction(pathIndex, () -> {
            setTurretFixedFar();
            turnFlywheelOnForBack();
            setHoodPosFar();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, farShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), farShootingPose.getHeading())
                .addParametricCallback(0.3, () -> { intake.stopIntakeNoReverse(); intake.railDown(); FIREWHEELS_ON = true; })
                .addParametricCallback(0, () -> follower.setMaxPower(0.8))
                .addParametricCallback(brakeT, () -> follower.setMaxPower(shootBrakePower));
        pathIndex++;

        pauseActions.put(pathIndex, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        lastPose = farShootingPose;
    }

    protected void shootAtCloseAndPark() {
        double brakeT = computeBrakeT(lastPose, closeShootAndParkPose);
        addEntryAction(pathIndex, () -> {
            setTurretFixedClosePark();
            setHoodPosClosePark();
            turnFlywheelOnForClosePark();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, closeShootAndParkPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootAndParkPose.getHeading())
                .addParametricCallback(0.3, () -> { intake.stopIntakeNoReverse(); intake.railDown(); FIREWHEELS_ON = true; })
                .addParametricCallback(0, () -> follower.setMaxPower(1))
                .addParametricCallback(brakeT, () -> follower.setMaxPower(0.6));
        pathIndex++;

        Pose nudge = new Pose(
                closeShootAndParkPose.getX(),
                closeShootAndParkPose.getY() - 0.5,
                closeShootAndParkPose.getHeading());
        pathBuilder
                .addPath(new BezierLine(closeShootAndParkPose, nudge))
                .setConstantHeadingInterpolation(closeShootAndParkPose.getHeading());
        pathIndex++;

        pauseActions.put(pathIndex - 1, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        addEntryAction(pathIndex - 1, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            HOOD_POS = 0;
            flywheelVel = 0;
            turret.setFixedAngleCustom(0);
        });

        lastPose = closeShootAndParkPose;
    }

    // --- Sorted shooting ---

    private void fireForShot(Motif.Shoot shot) {
        switch (shot) {
            case LEFT:      intake.shootLeft(); break;
            case RIGHT:     intake.shootRight(); break;
            case LEFT_ALL:  intake.shootAllLeft(); break;
            case RIGHT_ALL: intake.shootAllRight(); break;
        }
    }

    /** Adds the nudge path + sorted pause action at the current pathIndex. Used by all sorted shoot variants. */
    private void addSortedShootPause(Pose shootPose) {
        Pose nudge = new Pose(shootPose.getX(), shootPose.getY() - 0.5, shootPose.getHeading());
        pathBuilder
                .addPath(new BezierLine(shootPose, nudge))
                .setConstantHeadingInterpolation(shootPose.getHeading());
        pathIndex++;

        pauseActions.put(pathIndex - 1, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();

            intake.getAllColorSensorValues();
            intake.latchFrontColorSensor();
            intake.latchRightColorSensor();
            intake.latchLeftColorSensor();

            Motif possessed = new Motif(
                    intake.getLeftArtifact(),
                    intake.getFrontArtifact(),
                    intake.getRightArtifact()
            );

            List<Motif.Shoot> shotOrder;
            if (autoMotif != null) {
                shotOrder = possessed.getShotOrder(possessed, autoMotif);
            } else {
                shotOrder = List.of(Motif.Shoot.RIGHT_ALL);
            }

            if (shotOrder.size() == 1 && shotOrder.get(0) == Motif.Shoot.RIGHT_ALL) {
                intake.shootAllThree();
            } else {
                currentShotOrder = shotOrder;
                sortedShotStepTimer = new ElapsedTime();
                sortedShotState = SortedShotState.FIRING_FIRST;
            }
        });
        pauseDurations.put(pathIndex - 1, sortedShootPauseDuration);
    }

    protected void shootCloseSorted() {
        double brakeT = computeBrakeT(lastPose, closeShootingPose);
        addEntryAction(pathIndex, () -> {
            setTurretFixedClose();
            setHoodPosClose();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading())
                .addParametricCallback(0.3, () -> { intake.stopIntakeNoReverse(); intake.railDown(); FIREWHEELS_ON = true; })
                .addParametricCallback(0, () -> follower.setMaxPower(1))
                .addParametricCallback(brakeT, () -> { follower.setMaxPower(0.35); FIREWHEELS_ON = true; });
        pathIndex++;

        addSortedShootPause(closeShootingPose);
        lastPose = closeShootingPose;
    }

    protected void shootCloseSortedCurved() {
        double brakeT = computeBrakeT(lastPose, closeShootingPose);
        addEntryAction(pathIndex, () -> {
            setTurretFixedClose();
            setHoodPosClose();
        });
        pathBuilder
                .addPath(new BezierCurve(lastPose, curveIntake2, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading())
                .addParametricCallback(0.3, () -> { intake.stopIntakeNoReverse(); intake.railDown(); FIREWHEELS_ON = true; })
                .addParametricCallback(0, () -> follower.setMaxPower(1))
                .addParametricCallback(brakeT, () -> follower.setMaxPower(0.6));
        pathIndex++;

        addSortedShootPause(closeShootingPose);
        lastPose = closeShootingPose;
    }

    protected void shootCloseSortedAndPark() {
        double brakeT = computeBrakeT(lastPose, closeShootAndParkPose);
        addEntryAction(pathIndex, () -> {
            setTurretFixedClosePark();
            setHoodPosClosePark();
            turnFlywheelOnForClosePark();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, closeShootAndParkPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootAndParkPose.getHeading())
                .addParametricCallback(0.3, () -> { intake.stopIntakeNoReverse(); intake.railDown(); FIREWHEELS_ON = true; })
                .addParametricCallback(0, () -> follower.setMaxPower(1))
                .addParametricCallback(brakeT, () -> follower.setMaxPower(0.6));
        pathIndex++;

        addSortedShootPause(closeShootAndParkPose);

        addEntryAction(pathIndex - 1, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            HOOD_POS = 0;
            flywheelVel = 0;
            turret.setFixedAngleCustom(0);
        });

        lastPose = closeShootAndParkPose;
    }

    // =====================================================
    // PARK PATHS
    // =====================================================

    private void parkAt(Pose pose) {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            HOOD_POS = 0;
            flywheelVel = 0;
            turret.setFixedAngleCustom(0);
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, pose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), pose.getHeading());
        pathIndex++;

        lastPose = pose;
    }

    protected void parkAtFront() {
        parkAt(frontParkPose);
    }

    protected void parkAtBack() {
        parkAt(backParkPose);
    }

    protected void parkAtCustomPose(Pose park) {
        customParkPose = true;
        AutoPaths.setFrontParkPose(park);
        AutoPaths.setParkAngle(park.getHeading());

        addEntryAction(pathIndex, () -> {
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
