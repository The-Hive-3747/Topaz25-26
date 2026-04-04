package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathBuilder;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.utilities.AimbotValues;
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
    protected double toIntakePower = 0.5;
    protected double toIntakeBrakeTValue = 0.7;
    protected double intakePower = 1;
    protected double shootBrakeDistance = 18; // inches from shooting pose to start braking
    protected double shootBrakePower = 0.45;


    protected HashMap<Integer, Runnable> pauseActions = new HashMap<>();
    protected HashMap<Integer, Runnable> entryActions = new HashMap<>();
    protected HashSet<Integer> firedEntryActions = new HashSet<>();
    protected HashSet<Integer> firedPauseActions = new HashSet<>();

    protected ElapsedTime shootPauseTimer = null;
    protected boolean waitingToResume = false;
    protected double shootPauseDuration = 1.4; // tune this
    protected double gatePauseDuration = 1.1; // tune this
    protected HashMap<Integer, Double> pauseDurations = new HashMap<>();
    protected boolean hasShot = false;
    protected double settleTime = 0; //0.2
    protected double currentPauseDuration = 0;
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

        if (waitingToResume && !hasShot && shootPauseTimer != null && shootPauseTimer.seconds() > settleTime) {
            hasShot = true;
            Objects.requireNonNull(pendingShootAction).run();
        }

        if (waitingToResume && shootPauseTimer != null
                && shootPauseTimer.seconds() > currentPauseDuration) {
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

    /**
     * Computes turret angle from closeShootAndParkPose to the goal,
     * using the same formula as Turret.getAutoAimGoalAngle().
     */
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

    /**
     * Computes distance from closeShootAndParkPose to the aimbot goal,
     * then looks up hood position from the Aimbot table.
     */
    protected void setHoodPosClosePark() {
        HOOD_POS = getAimValuesForClosePark().hoodPos;
    }

    /**
     * Sets flywheel velocity based on distance from closeShootAndParkPose
     * to the aimbot goal, looked up from the Aimbot table.
     */
    protected void turnFlywheelOnForClosePark() {
        flywheelVel = getAimValuesForClosePark().velocity;
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
        pathBuilder.setGlobalDeceleration();
        pathIndex = 0;
        pauseActions.clear();
        pauseDurations.clear();
        entryActions.clear();
        firedEntryActions.clear();
        firedPauseActions.clear();
    }

    /**
     * Adds an entry action at the given index. If one already exists,
     * composes them so both run (existing first, then new).
     */
    protected void addEntryAction(int index, Runnable action) {
        Runnable existing = entryActions.get(index);
        if (existing != null) {
            entryActions.put(index, () -> { existing.run(); action.run(); });
        } else {
            entryActions.put(index, action);
        }
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
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intake1StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake1StartPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(toIntakePower));
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intake1StartPose, intake1EndPose))
                .setLinearHeadingInterpolation(intake1StartPose.getHeading(), intake1EndPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(intakePower));
        pathIndex++;

        lastPose = intake1EndPose;
    }

    protected void intake1Sidespike() {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intake1SidespikeStartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake1SidespikeStartPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(0.6));
        pathIndex++;

        pathBuilder
                .addPath(new BezierCurve(intake1SidespikeStartPose, intake1SidespikeCurvePose, intake1SidespikeEndPose))
                .setLinearHeadingInterpolation(intake1SidespikeStartPose.getHeading(), intake1SidespikeEndPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(1));
        pathIndex++;

        lastPose = intake1SidespikeEndPose;
    }

    protected void intake2() {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intake2StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake2StartPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(toIntakePower));
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intake2StartPose, intake2EndPose))
                .setLinearHeadingInterpolation(intake2StartPose.getHeading(), intake2EndPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(intakePower));
        pathIndex++;

        lastPose = intake2EndPose;
    }

    protected void intake3() {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intake3StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake3StartPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(toIntakePower));
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intake3StartPose, intake3EndPose))
                .setLinearHeadingInterpolation(intake3StartPose.getHeading(), intake3EndPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(intakePower));
        pathIndex++;

        lastPose = intake3EndPose;
    }

    protected void intakeHP() {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, intakeHPStartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intakeHPStartPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(toIntakePower));
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(intakeHPStartPose, intakeHPEndPose))
                .setLinearHeadingInterpolation(intakeHPStartPose.getHeading(), intakeHPEndPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(intakePower));

        pathIndex++;

        lastPose = intakeHPEndPose;
    }

    protected void intakeGate() {
        addEntryAction(pathIndex, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            intake.startIntake();
        });
        pathBuilder
                .addPath(new BezierLine(lastPose, gateIntakeStartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), gateIntakeStartPose.getHeading())
                .addParametricCallback(toIntakeBrakeTValue, () -> follower.setMaxPower(toIntakePower));
        pathIndex++;

        pathBuilder
                .addPath(new BezierLine(gateIntakeStartPose, gateIntakeEndPose))
                .setLinearHeadingInterpolation(gateIntakeStartPose.getHeading(), gateIntakeEndPose.getHeading())
                .addParametricCallback(0, () -> follower.setMaxPower(0.5));
        pathIndex++;

        pauseActions.put(pathIndex, () -> {});
        pauseDurations.put(pathIndex, gatePauseDuration);
        addEntryAction(pathIndex, () -> intake.stopIntake());

        lastPose = gateIntakeEndPose;
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

    /**
     * Appends a path to the close shooting pose and registers a pause
     * on the NEXT path index. When Pedro transitions past this path,
     * onUpdate() catches it and pauses before follower.update() runs.
     * Index layout example:
     *   pathIndex N:   drive to closeShootingPose (entry action preps turret/hood)
     *   pathIndex N+1: whatever comes next (pause registered here, shoot action runs)
     */
    private double computeBrakeT(Pose start, Pose end) {
        double dx = end.getX() - start.getX();
        double dy = end.getY() - start.getY();
        double pathLength = Math.sqrt(dx * dx + dy * dy);
        return Math.max(0, (pathLength - shootBrakeDistance) / pathLength);
    }

    protected void shootAtClose() {
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
                .addParametricCallback(brakeT, () -> follower.setMaxPower(0.35));
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

        // Dummy path so Pedro has a next segment to transition to, triggering the pause
        Pose nudge = new Pose(
                closeShootAndParkPose.getX(),
                closeShootAndParkPose.getY() - 0.5,
                closeShootAndParkPose.getHeading());
        pathBuilder
                .addPath(new BezierLine(closeShootAndParkPose, nudge))
                .setConstantHeadingInterpolation(closeShootAndParkPose.getHeading());
        pathIndex++;

        // Pause fires when Pedro transitions past the drive path
        pauseActions.put(pathIndex - 1, () -> {
            FIREWHEELS_ON = true;
            intake.turnIsShootingTrue();
            intake.shootAllThree();
        });

        // After the shoot pause ends, clean up like a park
        addEntryAction(pathIndex - 1, () -> {
            FIREWHEELS_ON = false;
            intake.firewheelsOff();
            HOOD_POS = 0;
            flywheelVel = 0;
            turret.setFixedAngleCustom(0);
        });

        lastPose = closeShootAndParkPose;
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

    protected void parkAtFront() {
        addEntryAction(pathIndex, () -> {
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
        addEntryAction(pathIndex, () -> {
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