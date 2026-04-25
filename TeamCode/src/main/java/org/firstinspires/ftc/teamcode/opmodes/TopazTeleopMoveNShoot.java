package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.NormalizedColorSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.DataLogger;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.utilities.ShooterKinematics;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import java.util.List;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name="shoot n move teleop")
public class TopazTeleopMoveNShoot extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(TopazTeleopMoveNShoot.class);

    {
        addComponents(
                flywheel = new Flywheel(),
                drive = new FieldCentricDrive(),
                intake = new Intake(),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower),
                aimbot = new Aimbot(),
                turret = new Turret(),
                limelight = new Relocalization(),
                //limelightComponent = new LimelightComponent(),
                dataLogger = new DataLogger(telemetry)
        );
    }


    //Relocalization limelight;
    DataLogger dataLogger;
    Intake intake;
    NormalizedColorSensor frontSensor;
    NormalizedColorSensor rightSensor;
    NormalizedColorSensor leftSensor;
    FieldCentricDrive drive;
    Aimbot aimbot;
    Turret turret;
    Relocalization limelight;
    Flywheel  flywheel;
    //TurretLights turretLights;
    private ElapsedTime looptime;
    private ElapsedTime relocalizeTimer = new ElapsedTime();
    private ShooterKinematics.ShotParameters shotParameters;
    private double relocalizeBreak = 1000;
    private double highestLooptime = 0;
    //private LimelightComponent limelightComponent;
    private double limelightCorrection = 0.0;
    double FLYWHEEL_VEL = 3200; //4000; //2000;//= 1300; // IN RPM
    double HOOD_POS;
    double INTAKE_POWER = 0.9;
    double INTAKE_SHOOTING_POWER = 0.65;
    double THREE_BALL_CURRENT = 6500.0;
    private boolean FLYWHEEL_ON = false;
    private boolean wasReadyToShoot = false;
    private boolean isIntakeOn = false;
    private boolean isIntakeReversed = false;
    private boolean isTransferOn = false;
    private boolean fireWhenReady = false;
    private boolean isFlipperOn = false;
    private boolean got3Balls = false;
    private boolean isRelocalized = false;
    private boolean limelightRelocalized = false;
    int FLYWHEEL_STEP = 50;
    private double FIRE_POWER = 0.9;
    private double slowModeMultiplier = 1;
    private double FLYWHEEL_CIRCUMFERENCE = Math.PI * 2 * (36/25.4); //36mm radius, 25.4mm to inches
    //hood friction slows exit vel between 0.5 and 0.75 of flywheel surface speed
    public static double HOOD_FRICTION_SPEED_FACTOR = 0.75;
    public static double HOOD_UP_TICKS = 7700;
    public static double HOOD_UP_DEG = 45;
    public static double HOOD_DOWN_TICKS = 1200;
    public static double HOOD_DOWN_DEG = 80;
    private static double HEIGHT_DIFF_ROBOT_TO_GOAL_IN = 28;

    private GoBildaPrismDriver prism;
    Follower follower;
    public Alliance alliance;
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public ElapsedTime lightTimer = new ElapsedTime();
    Button g2A;
    List<LynxModule> allHubs;


    //GraphManager graphManager = PanelsGraph.INSTANCE.getManager();
    double botDistance;

    @Override
    public void onInit() {
        follower = Constants.createFollower(hardwareMap);
        drive.setOffset(OpModeTransfer.currentPose.getHeading());
        follower.setStartingPose(OpModeTransfer.currentPose);
        follower.update();




        //prism = hardwareMap.get(GoBildaPrismDriver.class,"prism");
        //limelight = new Relocalization();
        //limelight.preInit();
        //limelightComponent = hardwareMap.get(LimelightComponent.class, "limelight");

        //turretLights = new TurretLights(hardwareMap, telemetry);

        Drawing.init();

        alliance = OpModeTransfer.alliance;
        Button g1Back = button(() -> gamepad1.back);
        Button g2Back = button(() -> gamepad2.back);
        Button g1DDown = button(() -> gamepad1.dpad_down);
        g1DDown.whenBecomesTrue(() -> flywheel.resetHoodEncoder());
        g2Back.whenBecomesTrue(() -> turret.zeroTurret());
        g1Back.whenBecomesTrue(() -> {
                    if (alliance == Alliance.BLUE){
                        alliance = Alliance.RED;
                        //turretLights.redAlliance();
                    } else{
                        alliance = Alliance.BLUE;
                        //turretLights.blueAlliance();
                    }
                    //turret.setAlliance(alliance);
                });
        looptime = new ElapsedTime();

        if (alliance == Alliance.BLUE){
            //turretLights.redAlliance();
        } else{
            //turretLights.blueAlliance();
        }


    }
    @Override
    public void onWaitForStart() {

    }

    /*public void relocalizeButton(){
        //limelight.update();
        if(limelight.isDataFresh() && !isRelocalized){
            follower.setPose(new Pose(limelight.getPedroPose().getX(), limelight.getPedroPose().getY(), limelight.getPedroPose().getHeading()));
            relocalizeTimer.reset();
            isRelocalized = true;
        }
        if(isRelocalized) {
            telemetry.addData("---------------------SUCESSFULLY RELOCALIZED------------------------", "");
            if (relocalizeTimer.milliseconds() >= relocalizeBreak) {
                isRelocalized = false;
            }
        }
    }*/

    @Override
    public void onStartButtonPressed() {
        //follower.startTeleOpDrive();

        if (OpModeTransfer.hasBeenTransferred == false) {
            turret.zeroTurret();
            flywheel.resetHoodEncoder();
        }

        turret.setAlliance(alliance);
        aimbot.setAlliance(alliance);
        turret.setTurretStateMoveNShoot();

        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        g2A = button(() -> gamepad2.a);

        Button gUp = button(() -> gamepad2.dpad_up || gamepad1.dpad_up);
        Button gDown = button(() -> gamepad2.dpad_down || gamepad1.dpad_down);

        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Left = button(() -> gamepad1.dpad_left);
        Button g2LB = button(() -> gamepad2.left_bumper || gamepad1.left_bumper);
        Button g2RB = button(() -> gamepad2.right_bumper || gamepad1.right_bumper);
        Button g1A = button(() -> gamepad1.a);
        Button g1B = button(() -> gamepad1.b);
        Button g1X = button(() -> gamepad1.x);
        Button g1Y = button(() -> gamepad1.y);

        Button g2RT = button(() -> gamepad2.right_trigger > 0.1);
        Button g1LT = button(() -> gamepad1.left_trigger > 0.1);
        Button g2LT = button(() -> gamepad2.left_trigger > 0.1);
        Button g1RT = button(() -> gamepad1.right_trigger > 0.1);


        gUp.whenBecomesTrue(() -> flywheel.increaseHood());
        gDown.whenBecomesTrue(() -> {
            flywheel.setHoodPower(-0.5);
        })
                .whenBecomesFalse(() -> {
                    flywheel.setHoodPower(0);
                    flywheel.resetHoodEncoder();
                });

        //g1X.whenBecomesTrue(() -> odoTurret.resetTurret());

        //g2LT.toggleOnBecomesTrue()
        g2LT.whenBecomesTrue(() ->{
            FLYWHEEL_ON = true;
            //intake.stopIntake();
            intake.railDown();
            intake.startRailDex();
            //intake.startRailDexTime();
        });
        //g2LT.whenBecomesFalse(() -> intake.startResetRailDex());//intake.resetRailDex());
                //.whenBecomesFalse(() -> intake.resetRailDex());
        g2RT.toggleOnBecomesTrue()
                .whenBecomesTrue( () -> intake.reverseIntake())
                .whenBecomesFalse(() -> intake.stopReverseIntake());

        g1Right.whenBecomesTrue(() -> turret.turretStateForward());

        g1Left.whenBecomesTrue(() -> turret.turretStateBackward());

        /*g1Y.toggleOnBecomesTrue()
                        .whenBecomesTrue(() -> turret.setTurretStateoff())
                        .whenBecomesFalse(() -> turret.setTurretStateAuto());*/

        g1RT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> slowModeMultiplier = 0.5)
                .whenBecomesFalse(() -> slowModeMultiplier = 1);
        g2B.whenBecomesTrue(() -> intake.turnAgitator());


        g2Y.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> FLYWHEEL_ON = true)
                .whenBecomesFalse(() -> FLYWHEEL_ON = false);

        /*g2RT.whenBecomesTrue(() -> {  //preset hood and velocity for position 1
                flywheel.setTargetVel(POSE_ONE_VEL);
                        });
        g2LT.whenBecomesTrue(() -> {  //preset hood and velocity for position 2
                flywheel.setTargetVel(POSE_TWO_VEL);
        });*/


        g2A.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    //intake.turnIsShootingFalse();
                    intake.startIntake();
                    isIntakeOn = true;
                })
                .whenBecomesFalse(() -> {
                    intake.stopIntake();
                    intake.resetRailDex();
                    isIntakeOn = false;
                });

        /*g2LT.toggleOnBecomesTrue() //reversing intake
                .whenBecomesTrue(() -> {
                    intakeMotor.setPower(-INTAKE_POWER);
                    fireWheelRight.setPower(-FIRE_POWER);
                    fireWheelLeft.setPower(-FIRE_POWER);
                    isIntakeReversed = true;
                })
                .whenBecomesFalse(() -> {
                    intakeMotor.setPower(0);
                    fireWheelRight.setPower(FIRE_POWER);
                    fireWheelLeft.setPower(FIRE_POWER);
                    isIntakeReversed = false;
                });*/

        g1A.whenBecomesTrue(() -> {
            if (alliance == Alliance.RED) {
                follower.setPose(new Pose(9.5, 9, Math.toRadians(90)));
            } else {
                follower.setPose(new Pose(134.5, 9, Math.toRadians(90)));
            }
        });

        /*g2X.toggleOnBecomesTrue() //firewheels on
                .whenBecomesTrue(() -> {
                    fireWheelLeft.setPower(FIRE_POWER);
                    fireWheelRight.setPower(FIRE_POWER);
                    isTransferOn = true;
                })
                .whenBecomesFalse(() -> {
                    fireWheelLeft.setPower(0);
                    fireWheelRight.setPower(0);
                    isTransferOn = false;
                });*/




        /*gUpOrDown.whenBecomesFalse(() -> { //HOOD ENCODER
                    flywheel.setHoodPower(0);
                    //flywheel.resetHoodEncoder();
                });*/

        g2LB.whenBecomesTrue(() -> flywheel.decrease());
        g2RB.whenBecomesTrue(() -> flywheel.increase());


        g1B.whenBecomesTrue(() -> {
            drive.setOffset(follower.getHeading());
        });
        g1X.whenBecomesTrue(() -> {
            if (limelight.isDataFresh()) {
                turret.setCurrentPose(follower.getPose(), follower.getVelocity(), limelight.getPedroPose().getHeading());
            }
        });

        /*gUp.whenBecomesTrue(() -> FLYWHEEL_VEL += 200); //OLD FLYWHEEL INCREASE
        gDown.whenBecomesTrue(() -> {
                if (FLYWHEEL_VEL - 200 < 0) {
            FLYWHEEL_VEL = 0;
        } else {
            FLYWHEEL_VEL -= 200;
        }
        });*/

        //g1X.whenBecomesTrue(() -> relocalizeButton());

        //g1LT.whenBecomesTrue(() -> turretLights.redAlliance());

        //turning on bulk mode to reduce critical loop time
        allHubs = hardwareMap.getAll(LynxModule.class);
        for(LynxModule hub : allHubs){
            hub.setBulkCachingMode(LynxModule.BulkCachingMode.AUTO);
        }

    }
    @Override
    public void onUpdate() {
        //this must occur when using manual caching or we see the same values each loop
        for(LynxModule module : allHubs){
            module.clearBulkCache();
        }
        looptime.reset();
        follower.update();
        drive.update(follower.getHeading(), slowModeMultiplier);


        dataLogger.setCurrentPose(follower.getPose());
        dataLogger.addAimbot(aimbot);
        dataLogger.addTurret(turret);
        //dataLogger.update();


        aimbot.setCurrentPose(follower.getPose(), follower.getVelocity());
        aimbot.update();
        FLYWHEEL_VEL = aimbot.getAimbotValues().velocity;
        HOOD_POS = aimbot.getAimbotValues().hoodPos;
        flywheel.setHoodGoalPos(HOOD_POS);

        shotParameters = ShooterKinematics.calculate3DMovingShot(follower.getPose().getX(),
                follower.getPose().getY(),
                follower.getVelocity().getXComponent(),
                follower.getVelocity().getYComponent(),
                aimbot.getGoalX(),
                aimbot.getGoalY(),
                HEIGHT_DIFF_ROBOT_TO_GOAL_IN,
                convertHoodTicksToDeg((int)HOOD_POS),
                FLYWHEEL_CIRCUMFERENCE);
        FLYWHEEL_VEL = shotParameters.flywheelRPM / HOOD_FRICTION_SPEED_FACTOR;

        if (FLYWHEEL_ON) {
            flywheel.setTargetVel(FLYWHEEL_VEL);
        } else {
            flywheel.setTargetVel(0);
        }


        /*if(intakeMotor.getCurrent(CurrentUnit.MILLIAMPS) > THREE_BALL_CURRENT){
            intakeMotor.setPower(0);
            got3Balls = true;
        }*/

        if(flywheel.readyToShoot() && !wasReadyToShoot && lightTimer.seconds() > 1.0 && flywheel.getVel() != 0){
            //turretLights.readyToShoot();
            wasReadyToShoot = true;
            lightTimer.reset();
        } else if (!flywheel.readyToShoot() && wasReadyToShoot && lightTimer.seconds() > 1.0) {
            wasReadyToShoot = false;
            //turretLights.notReadyToShoot();
            lightTimer.reset();
        }

        /*double currentHeading = follower.getHeading();
        limelightComponent.update(currentHeading);
        if (limelightComponent.hasValidBotPose()) {
            int tagId = limelightComponent.getAprilTagId();
            boolean isGoalTag = (tagId >= 1 && tagId <= 2);
            if (isGoalTag) {
              Pose limelightPose = new Pose(limelightComponent.getRobotX(),limelightComponent.getRobotY(),limelightComponent.getRobotHeading());
            }
       }*/



        limelight.update();
        BindingManager.update();
        flywheel.update();
        intake.update();

        if (limelight.isDataFresh()) {
            limelightRelocalized = true;
            limelightCorrection = limelight.getPedroPose().getHeading();
            turret.setCurrentPose(follower.getPose(), follower.getVelocity(), limelight.getPedroPose().getHeading());
        } else {
            //limelightCorrection = 0.0;
            turret.setCurrentPose(follower.getPose(), follower.getVelocity(), limelightCorrection);
        }


        // UNCOMMENT TO ENABLE SHOOT ON THE MOVE. NEEDS TO BE TESTED.
        turret.shootOnTheMove(follower.getVelocity());

        turret.setCurrentPose(follower.getPose(), follower.getVelocity(), 0);
        //turret.setTurretShootAngle(Math.toDegrees(shotParameters.headingRadians));
        //turret.setTurretShootAngle(turret.convertTurretHeading(shotParameters.headingRadians)); //more recent one
        turret.setTurretOnTheMoveInRads(shotParameters.headingRadians);
        turret.update();


        Drawing.drawOnlyCurrentWithTurretAndGoal(follower,
                Math.toRadians(turret.getTurretAngle()) + follower.getHeading() + Math.toRadians(180),
                Math.toRadians(turret.getTurretGoalNotInLimits()) + follower.getHeading() + Math.toRadians(180)
        );
        Drawing.drawDebug(follower);

        Drawing.drawOnlyCurrent(follower);

        if (looptime.milliseconds() > highestLooptime) {
            highestLooptime = looptime.milliseconds();
        }



        //graphManager.addData("flywheel velocity", flywheel.getVel());
        //graphManager.addData("flywheel goal velocity", flywheel.getFlywheelGoal());
        //graphManager.addData("flywheel power", flywheel.getPower());
        //graphManager.update();

        //panelsTelemetry.addData("Intake Current (mA)", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        panelsTelemetry.addData("hood position", flywheel.getHoodPos());
        panelsTelemetry.addData("hood goal", flywheel.getHoodGoal());
        panelsTelemetry.addData("hub number", allHubs.toArray().length);
        panelsTelemetry.addData("flywheel velocity", flywheel.getVel());
        panelsTelemetry.addData("flywheel goal velocity", flywheel.getFlywheelGoal());
        panelsTelemetry.addData("Critical loop time", looptime);
        panelsTelemetry.addData("Highset loop time", highestLooptime);
        panelsTelemetry.addData("flywheel power", flywheel.getPower());
        panelsTelemetry.addData("LeftFlyWheel Current (mA)", flywheel.getCurrentLeft());
        panelsTelemetry.addData("Right Flywheel Current (mA)", flywheel.getCurrentRight());
        panelsTelemetry.addData("bot Distance", aimbot.getBotDistance());
        panelsTelemetry.addData("bot values", aimbot.getAimbotValues());


        /*telemetry.addData("limelight correction", limelightCorrection);
        telemetry.addData("limelight relocalized", limelightRelocalized);
        telemetry.addData("hood pose", flywheel.getHoodPosition());
        //telemetry.addData("Limelight fresh",limelight.isDataFresh());
        //telemetry.addData("Intake Current (mA)", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("LeftFlyWheel Current (mA)", flywheel.getCurrentLeft());
        telemetry.addData("Right Flywheel Current (mA)", flywheel.getCurrentRight());
        telemetry.addData("looptime (ms)", looptime.milliseconds());
        telemetry.addData("highest looptime (ms)", highestLooptime);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Is Intake on: ", isIntakeOn);
        telemetry.addData("Is Intake Reversed: ", isIntakeReversed);
        telemetry.addData("Is Flywheel on: ", FLYWHEEL_ON);
        telemetry.addData("Is Transfer on: ", isTransferOn);*/
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        OpModeTransfer.hasBeenTransferred = false;
    }

    public int convertHoodTicksToDeg(int hoodTicks){
        return (int) Math.round(HOOD_DOWN_DEG + (hoodTicks + HOOD_DOWN_TICKS) * (HOOD_UP_DEG - HOOD_DOWN_DEG) / (HOOD_UP_TICKS - HOOD_DOWN_TICKS));
    }
    public double calcExitVel(double flywheelRPM){
        return flywheelRPM * FLYWHEEL_CIRCUMFERENCE * HOOD_FRICTION_SPEED_FACTOR;
    }
}
