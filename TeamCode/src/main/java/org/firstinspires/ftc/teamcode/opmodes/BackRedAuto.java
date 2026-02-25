package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.intake1;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.intake2;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.intake3;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.lineUpForIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.lineUpForIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.lineUpForIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.park;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.startAngle;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.startingPose;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.BackAutoPaths.toShootFromStart;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.subsystems.Aimbot;
import org.firstinspires.ftc.teamcode.subsystems.Hood;
import org.firstinspires.ftc.teamcode.subsystems.TurretLights;
import org.firstinspires.ftc.teamcode.utilities.Alliance;
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
    boolean FLYWHEEL_ON = false;
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


        turretLights = new TurretLights(hardwareMap, telemetry);

        if (BackAutoPaths.getAlliance() == Alliance.RED) {
            turretLights.redAlliance();
        } else {
            turretLights.blueAlliance();
            //light.setColor(Light.COLOR_BLUE);
        }

        autonomous = new SequentialGroup(
                new ParallelGroup(
                        //intake.startIntake,
                        intake.fastIntake,
                        this.startAimbotFlywheel,
                        //flywheel.startFlywheel,
                        //turret.setTurretOff,
                        //turret.setTurretAuto,
                        turret.setTurretFixed,

                        new FollowPath(toShootFromStart)
                ),
                new Delay(0.8),
                new ParallelGroup(
                        //turret.setTurretAuto,
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer,
                        intake.slowIntake
                ),
                new ParallelGroup(
                        new FollowPath(lineUpForIntake1),
                        intake.stopTransfer,
                        //intake.startIntake
                        intake.fastIntake
                ),
                new FollowPath(intake1),
                new Delay(1),
                new FollowPath(toShootFromIntake1),
                new Delay(0.5),
                new ParallelGroup(
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer,
                        intake.slowIntake
                ),
                new ParallelGroup(
                        new FollowPath(lineUpForIntake2),
                        intake.stopTransfer,
                        //intake.startTransfer
                        intake.fastIntake
                ),
                new Delay(0.2),
                new FollowPath(intake2),
                new Delay(1),
                new FollowPath(toShootFromIntake2),
                new Delay(0.3),
                new ParallelGroup(
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer,
                        intake.slowIntake
                ),
                new ParallelGroup(
                        new FollowPath(lineUpForIntake3),
                        intake.stopTransfer,
                        //intake.startIntake
                        intake.fastIntake
                ),
                new Delay(0.2),
                new FollowPath(intake3), //setFlywheelVelFinal),
                new Delay(0.3),
                new FollowPath(toShootFromIntake3).and(intake.startTransfer),
                new Delay(0.3),
                new ParallelGroup(
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer,
                        intake.slowIntake
                ),
                new ParallelGroup(
                        new InstantCommand(
                                () -> flywheel.setHoodGoalPos(0)
                        ),
                        turret.setTurretForward,
                        flywheel.stopFlywheel,
                        intake.stopIntake,
                        intake.stopTransfer,
                        new FollowPath(park)
                )
        );

        turret.zeroTurret();
    }

    @Override
    public void onWaitForStart() {
        //flywheel.setHoodGoalPos(1247);
        //flywheel.update();
        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        //turret.zeroTurret();
        flywheel.resetHoodEncoder();
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
        flywheel.setHoodGoalPos(HOOD_POS);
        flywheel.setTargetVel(FLYWHEEL_VEL);
        //flywheel.setTargetVel(FLYWHEEL_VEL);
        //flywheel.setTargetVel(0);

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
