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

import dev.nextftc.extensions.pedro.PedroComponent;
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

    // Shoot pause/resume
    protected ElapsedTime shootPauseTimer = new ElapsedTime();
    protected boolean waitingToResume = false;
    protected double shootPauseDuration = 1; // tune this

    // Path building
    protected PathBuilder pathBuilder;
    protected PathChain allPaths;

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
    }

    @Override
    public void onUpdate() {
        turret.setTurretStateFixed();
        turret.update();
        follower.update();

        flywheel.setHoodGoalPos(HOOD_POS);
        flywheel.setTargetVel(flywheelVel);

        Drawing.drawOnlyCurrent(follower);

        if (FIREWHEELS_ON) {
            intake.runFireWheels();
        }

        // Resume path following after shoot pause
        if (waitingToResume && shootPauseTimer.seconds() > shootPauseDuration) {
            waitingToResume = false;
            follower.resumePathFollowing();
        }

        telemetry.addData("pose", follower.getPose());
        telemetry.addData("chain index", follower.getChainIndex());

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
        pathBuilder
                .addPath(new BezierLine(lastPose, intake1StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake1StartPose.getHeading())
                .addParametricCallback(0.0, () -> {
                    FIREWHEELS_ON = false;
                    intake.firewheelsOff();
                })
                .addParametricCallback(0.5, () -> intake.startIntake())

                .addPath(new BezierLine(intake1StartPose, intake1EndPose))
                .setLinearHeadingInterpolation(intake1StartPose.getHeading(), intake1EndPose.getHeading())
                .addParametricCallback(0.95, () -> intake.stopIntake());

        lastPose = intake1EndPose;
    }

    protected void intake2() {
        pathBuilder
                .addPath(new BezierLine(lastPose, intake2StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake2StartPose.getHeading())
                .addParametricCallback(0.0, () -> {
                    FIREWHEELS_ON = false;
                    intake.firewheelsOff();
                })
                .addParametricCallback(0.5, () -> intake.startIntake())

                .addPath(new BezierLine(intake2StartPose, intake2EndPose))
                .setLinearHeadingInterpolation(intake2StartPose.getHeading(), intake2EndPose.getHeading())
                .addParametricCallback(0.95, () -> intake.stopIntake());

        lastPose = intake2EndPose;
    }

    protected void intake3() {
        pathBuilder
                .addPath(new BezierLine(lastPose, intake3StartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intake3StartPose.getHeading())
                .addParametricCallback(0.0, () -> {
                    FIREWHEELS_ON = false;
                    intake.firewheelsOff();
                })
                .addParametricCallback(0.5, () -> intake.startIntake())

                .addPath(new BezierLine(intake3StartPose, intake3EndPose))
                .setLinearHeadingInterpolation(intake3StartPose.getHeading(), intake3EndPose.getHeading())
                .addParametricCallback(0.95, () -> intake.stopIntake());

        lastPose = intake3EndPose;
    }

    protected void intakeHP() {
        pathBuilder
                .addPath(new BezierLine(lastPose, intakeHPStartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), intakeHPStartPose.getHeading())
                .addParametricCallback(0.0, () -> {
                    FIREWHEELS_ON = false;
                    intake.firewheelsOff();
                })
                .addParametricCallback(0.5, () -> intake.startIntake())

                .addPath(new BezierLine(intakeHPStartPose, intakeHPEndPose))
                .setLinearHeadingInterpolation(intakeHPStartPose.getHeading(), intakeHPEndPose.getHeading())
                .addParametricCallback(0.95, () -> intake.stopIntake());

        lastPose = intakeHPEndPose;
    }

    protected void openGate() {
        pathBuilder
                .addPath(new BezierLine(lastPose, openGateStartPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), openGateStartPose.getHeading())
                .addParametricCallback(0.0, () -> intake.stopIntake())

                .addPath(new BezierLine(openGateStartPose, openGateEndPose))
                .setLinearHeadingInterpolation(openGateStartPose.getHeading(), openGateEndPose.getHeading());

        lastPose = openGateEndPose;
    }

    protected void shootAtClose() {
        pathBuilder
                .addPath(new BezierLine(lastPose, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading())
                .addParametricCallback(0.2, this::setTurretFixedClose)
                .addParametricCallback(0.3, () -> intake.railDown())
                .addParametricCallback(0.5, this::setHoodPosClose)
                .addParametricCallback(0.9, () -> {
                    follower.pausePathFollowing();
                    FIREWHEELS_ON = true;
                    intake.turnIsShootingTrue();
                    intake.shootAllThree();
                    shootPauseTimer.reset();
                    waitingToResume = true;
                });

        lastPose = closeShootingPose;
    }

    protected void shootAtCloseCurved() {
        pathBuilder
                .addPath(new BezierCurve(lastPose, curveIntake2, closeShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), closeShootingPose.getHeading())
                .addParametricCallback(0.2, this::setTurretFixedClose)
                .addParametricCallback(0.3, () -> intake.railDown())
                .addParametricCallback(0.5, this::setHoodPosClose)
                .addParametricCallback(1.0, () -> {
                    follower.pausePathFollowing();
                    FIREWHEELS_ON = true;
                    intake.turnIsShootingTrue();
                    intake.shootAllThree();
                    shootPauseTimer.reset();
                    waitingToResume = true;
                });

        lastPose = closeShootingPose;
    }

    protected void shootAtFar() {
        pathBuilder
                .addPath(new BezierLine(lastPose, farShootingPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), farShootingPose.getHeading())
                .setVelocityConstraint(0.7)
                .addParametricCallback(0.2, this::setTurretFixedFar)
                .addParametricCallback(0.3, () -> intake.railDown())
                .addParametricCallback(0.5, this::setHoodPosFar)
                .addParametricCallback(1.0, () -> {
                    follower.pausePathFollowing();
                    FIREWHEELS_ON = true;
                    intake.turnIsShootingTrue();
                    intake.shootAllThree();
                    shootPauseTimer.reset();
                    waitingToResume = true;
                });

        lastPose = farShootingPose;
    }

    protected void parkAtFront() {
        pathBuilder
                .addPath(new BezierLine(lastPose, frontParkPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), frontParkPose.getHeading())
                .addParametricCallback(0.0, () -> {
                    FIREWHEELS_ON = false;
                    intake.firewheelsOff();
                    HOOD_POS = 0;
                    flywheelVel = 0;
                    turret.setFixedAngleCustom(0);
                });

        lastPose = frontParkPose;
    }

    protected void parkAtBack() {
        pathBuilder
                .addPath(new BezierLine(lastPose, backParkPose))
                .setLinearHeadingInterpolation(lastPose.getHeading(), backParkPose.getHeading())
                .addParametricCallback(0.0, () -> {
                    FIREWHEELS_ON = false;
                    intake.firewheelsOff();
                    HOOD_POS = 0;
                    flywheelVel = 0;
                    turret.setFixedAngleCustom(0);
                });

        lastPose = backParkPose;
    }

    protected void parkAtCustomPose(Pose park) {
        customParkPose = true;
        AutoPaths.setFrontParkPose(park);
        AutoPaths.setParkAngle(park.getHeading());

        pathBuilder
                .addPath(new BezierLine(lastPose, park))
                .setLinearHeadingInterpolation(lastPose.getHeading(), park.getHeading())
                .addParametricCallback(0.0, () -> {
                    turret.setTurretAngle(0);
                    intake.firewheelsOff();
                    flywheelVel = 0;
                });

        lastPose = park;
    }
}