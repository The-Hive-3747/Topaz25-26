package org.firstinspires.ftc.teamcode.opmodes;

import static dev.nextftc.bindings.Bindings.button;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.FieldCentricDrive;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Relocalization;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
import org.firstinspires.ftc.teamcode.subsystems.TurretLights;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.DataLogger;
import org.firstinspires.ftc.teamcode.utilities.Drawing;
import org.firstinspires.ftc.teamcode.utilities.GoBildaPrismDriver;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.slf4j.Logger;
import org.slf4j.LoggerFactory;

import dev.nextftc.bindings.BindingManager;
import dev.nextftc.bindings.Button;
import dev.nextftc.core.components.BindingsComponent;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@TeleOp(name="topaz teleop")
public class TopazTeleop extends NextFTCOpMode {
    private static final Logger log = LoggerFactory.getLogger(TopazTeleop.class);

    {
        addComponents(
                flywheel = new Flywheel(),
                drive = new FieldCentricDrive(),
                intake = new Intake(),
                BindingsComponent.INSTANCE,
                new PedroComponent(Constants::createFollower)
                //aimbot = new Aimbot()
                //turret = new Turret(),
                //limelight = new Relocalization()
                //limelightComponent = new LimelightComponent(),
                //dataLogger = new DataLogger(telemetry)
        );
    }
    //Relocalization limelight;
    //DataLogger dataLogger;
    //Turret turret;
    Intake intake;
    FieldCentricDrive drive;
    //Aimbot aimbot;
    //Relocalization limelight;
    Flywheel  flywheel;
    //TurretLights turretLights;
    private ElapsedTime looptime;
    private ElapsedTime relocalizeTimer = new ElapsedTime();
    private double relocalizeBreak = 1000;
    private double highestLooptime = 0;
    //private LimelightComponent limelightComponent;
    double FLYWHEEL_VEL = 2000;//= 1300; // IN RPM
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
    int FLYWHEEL_STEP = 50;
    private double FIRE_POWER = 0.9;
    private double slowModeMultiplier = 1;
    double FLIPPER_FIRE_POS = 0.1;
    double FLIPPER_NO_FIRE_POS = 0.52;
    private GoBildaPrismDriver prism;
    Follower follower;
    public Alliance alliance;
    TelemetryManager panelsTelemetry = PanelsTelemetry.INSTANCE.getTelemetry();
    public ElapsedTime lightTimer = new ElapsedTime();

    Button g2A;

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
        //g2Back.whenBecomesTrue(() -> turret.zeroTurret());
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
            //turret.zeroTurret();
            flywheel.resetHoodEncoder();
        }

        //turret.setAlliance(alliance);
        //aimbot.setAlliance(alliance);



        Button g2X = button(() -> gamepad2.x);
        Button g2Y = button(() -> gamepad2.y);
        Button g2B = button(() -> gamepad2.b);
        g2A = button(() -> gamepad2.a);
        Button g2RT = button(() -> gamepad2.right_trigger > 0.1);

        Button gUp = button(() -> gamepad2.dpad_up || gamepad1.dpad_up);
        Button gDown = button(() -> gamepad2.dpad_down || gamepad1.dpad_down);
        Button gUpOrDown = gUp.or(gDown);

        Button g1Right = button(() -> gamepad1.dpad_right);
        Button g1Left = button(() -> gamepad1.dpad_left);
        Button g2LB = button(() -> gamepad2.left_bumper);
        Button g2RB = button(() -> gamepad2.right_bumper);
        Button g1A = button(() -> gamepad1.a);
        Button g1B = button(() -> gamepad1.b);
        Button g1X = button(() -> gamepad1.x);
        Button g1Y = button(() -> gamepad1.y);

        Button g1LT = button(() -> gamepad1.left_trigger > 0.1);
        Button g2LT = button(() -> gamepad2.left_trigger > 0.1);
        Button g1RT = button(() -> gamepad1.right_trigger > 0.1);



        //g1X.whenBecomesTrue(() -> odoTurret.resetTurret());

        g1LT.toggleOnBecomesTrue()
                .whenBecomesTrue(() ->{
                    FLYWHEEL_ON = true;
                    intake.stopIntake();
                    intake.startRailDex();
                })
                .whenBecomesFalse(() -> intake.resetRailDex());
        //g1Right.whenBecomesTrue(() -> turret.turretStateForward());

        //g1Left.whenBecomesTrue(() -> turret.turretStateBackward());

        /*g1Y.toggleOnBecomesTrue()
                        .whenBecomesTrue(() -> turret.setTurretStateoff())
                        .whenBecomesFalse(() -> turret.setTurretStateAuto());*/

        g1RT.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> slowModeMultiplier = 0.5)
                .whenBecomesFalse(() -> slowModeMultiplier = 1);



        g2Y.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> FLYWHEEL_ON = true)
                .whenBecomesFalse(() -> FLYWHEEL_ON = false);


        g2A.toggleOnBecomesTrue()
                .whenBecomesTrue(() -> {
                    intake.startIntake();
                    isIntakeOn = true;
                })
                .whenBecomesFalse(() -> {
                    intake.stopIntake();
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
            FrontAutoPathsOld.alliance = alliance;
            FrontAutoPathsOld.generatePaths(follower);
            Pose resetPose = new Pose(FrontAutoPathsOld.startingPose.getX(), FrontAutoPathsOld.startingPose.getY(), FrontAutoPathsOld.startAngle);
            follower.setPose(resetPose);
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


        /*g2RT.whenTrue(() -> {
                got3Balls = false;
                fireWhenReady = false;
                intakeMotor.setPower(INTAKE_SHOOTING_POWER);
                flipper.setPosition(FLIPPER_FIRE_POS);
                isFlipperOn = true;
                })
                .whenBecomesFalse(() -> {
                    flipper.setPosition(FLIPPER_NO_FIRE_POS);
                    isFlipperOn = false;
                });

        g2B.whenBecomesTrue(() -> {
            fireWhenReady = true;
            intakeMotor.setPower(INTAKE_SHOOTING_POWER);
            })
                .whenBecomesFalse(() -> {
                   fireWhenReady = false;
                   flipper.setPosition(FLIPPER_NO_FIRE_POS);
                });*/



        /*gUpOrDown.whenBecomesFalse(() -> {
                    flywheel.setHoodPower(0);
                    //flywheel.resetHoodEncoder();
                });*/

        //gUp.whenBecomesTrue(() -> flywheel.increaseHood());
        //gDown.whenBecomesTrue(() -> flywheel.decreaseHood());

        g2LB.whenBecomesTrue(() -> flywheel.decrease());
        g2RB.whenBecomesTrue(() -> flywheel.increase());

        g1B.whenBecomesTrue(() -> {
            drive.setOffset(follower.getHeading());

        });

        //g1X.whenBecomesTrue(() -> relocalizeButton());


        //g1LT.whenBecomesTrue(() -> turretLights.redAlliance());

    }
    @Override
    public void onUpdate() {
        drive.update(follower.getHeading(), slowModeMultiplier);
        looptime.reset();
        follower.update();
        //dataLogger.update();

        /*aimbot.setCurrentPose(follower.getPose(), follower.getVelocity());
        aimbot.update();
        FLYWHEEL_VEL = aimbot.getAimbotValues().velocity;
        HOOD_POS = aimbot.getAimbotValues().hoodPos;
        flywheel.setHoodGoalPos(HOOD_POS);*/
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



        //limelight.update();
        BindingManager.update();
        flywheel.update();
        intake.update();
        //turret.setCurrentPose(follower.getPose(), follower.getVelocity());

        // UNCOMMENT TO ENABLE SHOOT ON THE MOVE. NEEDS TO BE TESTED.
        // turret.shootOnTheMove(follower.getVelocity());

        //turret.update();
        if (fireWhenReady){
            if (flywheel.readyToShoot()){
                //flipper.setPosition(FLIPPER_FIRE_POS);
            }else{
                //flipper.setPosition(FLIPPER_NO_FIRE_POS);
            }
        }
        /*Drawing.drawOnlyCurrentWithTurretAndGoal(follower,
                Math.toRadians(turret.getTurretAngle()) + follower.getHeading() + Math.toRadians(180),
                Math.toRadians(turret.getTurretGoalNotInLimits()) + follower.getHeading() + Math.toRadians(180)
        );*/
        //Drawing.drawDebug(follower);

        if (looptime.milliseconds() > highestLooptime) {
            highestLooptime = looptime.milliseconds();
        }
        //graphManager.addData("flywheel velocity", flywheel.getVel());
        //graphManager.addData("flywheel goal velocity", flywheel.getFlywheelGoal());
        //graphManager.addData("flywheel power", flywheel.getPower());
        //graphManager.update();

        //panelsTelemetry.addData("Intake Current (mA)", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        panelsTelemetry.addData("flywheel velocity", flywheel.getVel());
        panelsTelemetry.addData("flywheel goal velocity", flywheel.getFlywheelGoal());
        panelsTelemetry.addData("flywheel power", flywheel.getPower());


        //telemetry.addData("Limelight fresh",limelight.isDataFresh());
        //telemetry.addData("Intake Current (mA)", intakeMotor.getCurrent(CurrentUnit.MILLIAMPS));
        telemetry.addData("looptime (ms)", looptime.milliseconds());
        telemetry.addData("highest looptime (ms)", highestLooptime);
        telemetry.addData("pose", follower.getPose());
        telemetry.addData("Is Intake on: ", isIntakeOn);
        telemetry.addData("Is Intake Reversed: ", isIntakeReversed);
        telemetry.addData("Is Flywheel on: ", FLYWHEEL_ON);
        telemetry.addData("Is Transfer on: ", isTransferOn);
        //telemetry.update();
        panelsTelemetry.update(telemetry);
    }

    @Override
    public void onStop() {
        OpModeTransfer.hasBeenTransferred = false;
    }
}
