package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.*;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

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
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import com.pedropathing.follower.Follower;
import dev.nextftc.ftc.NextFTCOpMode;

@Autonomous(name = "back red auto")
public class BackRedAuto extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                aimbot = new Aimbot(),
                turret = new Turret()
                //light = new Light()
        );
    }
    CommandGroup autonomous;
    //Light light;
    Aimbot aimbot;
    Turret turret;
    Flywheel flywheel;
    TurretLights turretLights;
    Intake intake;
    TelemetryManager telemetryM;
    Follower follower;
    double FLYWHEEL_VEL;
    double HOOD_POS;
    static boolean FLYWHEEL_ON = false;
    boolean FLIPPER_MOVE = false;


    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        BackAutoPaths.alliance = Alliance.RED;
        BackAutoPaths.generatePaths(PedroComponent.follower());

        follower = Constants.createFollower(hardwareMap);
        follower.setStartingPose(new Pose(startingPose.getX(), startingPose.getY(), startAngle));
        follower.update();

        turret.setAlliance(Alliance.RED);
        aimbot.setAlliance(Alliance.RED);
        turret.setFixedAngle(Turret.AUTON_RED_SHOOT_ANGLE);

        Drawing.init();


        //turretLights = new TurretLights(hardwareMap, telemetry);

        /*if (FrontAutoPaths.getAlliance() == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
            //light.setColor(Light.COLOR_BLUE);
        }*/

        autonomous = new SequentialGroup(
                new ParallelGroup(
                        //intake.startIntake,
                        startAimbotFlywheel,
                        //this.startAimbotFlywheel,
                        //flywheel.startFlywheel,
                        //turret.setTurretOff,
                        //turret.setTurretAuto,
                        //turret.setTurretFixed,
                        new FollowPath(toShootFromStart),
                        intake.railDownAuto
                ),
                new Delay(1.5),
                new ParallelGroup(
                        //turret.setTurretAuto,
                        //flywheel.resetShotTimer,
                        intake.shootAllThree
                        //intake.startTransfer,
                        //intake.slowIntake
                ),
                //new Delay(5.0),
                new ParallelGroup(
                        intake.firewheelsOff,
                        //intake.resetRailDex,
                        new FollowPath(lineUpForIntake1),
                        //intake.stopTransfer,
                        intake.startIntake
                        //intake.fastIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake1)
                ),
                new Delay(1),
                //new Delay(0.5),
                new FollowPath(toShootFromIntake1),
                intake.stopIntake,
                new ParallelGroup(
                        //flywheel.resetShotTimer,
                        intake.shootAllThree
                        //intake.startTransfer,
                        //intake.slowIntake
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        //intake.resetRailDex,
                        new FollowPath(lineUpForIntake2),
                        intake.startIntake
                        //intake.stopTransfer,
                        //intake.startTransfer
                        //intake.fastIntake
                ),
                //new Delay(0.2),
                new FollowPath(intake2),
                new Delay(1),
                intake.stopIntake,
                new FollowPath(toShootFromIntake2),
                new Delay(0.3),
                new ParallelGroup(
                        //flywheel.resetShotTimer,
                        intake.shootAllThree
                        //intake.startTransfer,
                        //intake.slowIntake
                ),
                //intake.startIntake,
                new ParallelGroup(
                        intake.firewheelsOff,
                        //intake.resetRailDex,
                        new FollowPath(lineUpForIntake3),
                        intake.startIntake
                ),
                //new Delay(0.2),
                new FollowPath(intake3), //setFlywheelVelFinal),
                new Delay(1.3),
                intake.stopIntake,
                new FollowPath(toShootFromIntake3),
                new Delay(0.3),
                new ParallelGroup(
                        //flywheel.resetShotTimer,
                        intake.shootAllThree
                        //intake.startIntake
                        //intake.slowIntake
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        /*new InstantCommand(
                                () -> flywheel.setHoodGoalPos(0)
                        ),*/
                        //turret.setTurretForward,
                        //intake.resetRailDex,
                        flywheel.stopFlywheel,
                        //intake.stopIntake,
                        //intake.stopTransfer,
                        new FollowPath(park)
                )
        );

        turret.zeroTurret();
    }

    @Override
    public void onWaitForStart() {
        //flywheel.setHoodGoalPos(1247);
        //flywheel.update();
        Drawing.drawOnlyCurrent(follower);
        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        //turret.zeroTurret();
        flywheel.resetHoodEncoder();
        FLYWHEEL_ON = true;
        //intake.startIntake();
        autonomous.schedule();
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

        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.addData("aimbot pose", follower.getPose());
        flywheel.update();
        intake.update();
        telemetry.update();
    }

    @Override
    public void onStop() {
        OpModeTransfer.currentPose = PedroComponent.follower().getPose();
        OpModeTransfer.alliance = Alliance.RED;
        OpModeTransfer.hasBeenTransferred = true;
    }
    public Command startAimbotFlywheel = new InstantCommand(
            () -> FLYWHEEL_ON = true
    );


}
