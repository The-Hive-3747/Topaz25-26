package org.firstinspires.ftc.teamcode.opmodes;

import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.intake1;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.intake2;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.intake3;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.lineUpForIntake1;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.lineUpForIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.lineUpForIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.lineUpForOpenGate;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.openGate;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.park;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.startAngle;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.startingPose;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.toShootFromIntake2;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.toShootFromIntake3;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.toShootFromOpenGate;
import static org.firstinspires.ftc.teamcode.opmodes.FrontAutoPathsOld.toShootFromStart;

import com.bylazar.telemetry.PanelsTelemetry;
import com.bylazar.telemetry.TelemetryManager;
import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.utilities.Alliance;
import org.firstinspires.ftc.teamcode.utilities.Light;
import org.firstinspires.ftc.teamcode.utilities.OpModeTransfer;
import org.firstinspires.ftc.teamcode.pathing.Constants;
import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Turret;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.CommandGroup;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.extensions.pedro.FollowPath;
import dev.nextftc.extensions.pedro.PedroComponent;
import dev.nextftc.ftc.NextFTCOpMode;

@Disabled
@Autonomous(name = "front red auto")
public class FrontRedAutoOld extends NextFTCOpMode {
    {
        addComponents(
                new PedroComponent(Constants::createFollower),
                flywheel = new Flywheel(),
                intake = new Intake(),
                turret = new Turret(),
                light = new Light()
        );
    }
    CommandGroup autonomous;
    Light light;
    Turret turret;
    Flywheel flywheel;
    Intake intake;
    TelemetryManager telemetryM;


    @Override
    public void onInit() {
        telemetryM = PanelsTelemetry.INSTANCE.getTelemetry();
        FrontAutoPathsOld.alliance = Alliance.RED;
        FrontAutoPathsOld.generatePaths(PedroComponent.follower());
        PedroComponent.follower().setStartingPose(new Pose(startingPose.getX(), startingPose.getY(), startAngle));

        turret.setAlliance(Alliance.RED);

        if (FrontAutoPathsOld.getAlliance() == Alliance.RED) {
            light.setColor(Light.COLOR_RED);
        } else {
            light.setColor(Light.COLOR_BLUE);
        }

        autonomous = new SequentialGroup(
                new ParallelGroup(
                        intake.startIntake,
                        flywheel.startFlywheel,
                        turret.setTurretOff,
                        new FollowPath(toShootFromStart)
                ),
                new Delay(1.3),
                new ParallelGroup(
                        turret.setTurretAuto,
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer
                ),
                new ParallelGroup(
                        new FollowPath(lineUpForIntake1),
                        intake.stopTransfer
                ),
                new FollowPath(intake1),
                new Delay(1),
                new FollowPath(lineUpForOpenGate),
                new FollowPath(openGate),
                new Delay(0.5),
                new FollowPath(toShootFromOpenGate),
                new Delay(0.5),
                new ParallelGroup(
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer
                ),
                new ParallelGroup(
                        new FollowPath(lineUpForIntake2),
                        intake.stopTransfer
                ),
                new Delay(0.2),
                new FollowPath(intake2),
                new Delay(1),
                new FollowPath(toShootFromIntake2),
                new Delay(0.3),
                new ParallelGroup(
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer
                ),
                new ParallelGroup(
                        new FollowPath(lineUpForIntake3),
                        intake.stopTransfer
                ),
                new Delay(0.2),
                new FollowPath(intake3),
                new Delay(0.3),
                new FollowPath(toShootFromIntake3),
                new Delay(0.3),
                new ParallelGroup(
                        flywheel.resetShotTimer,
                        flywheel.shootAllThree,
                        intake.startTransfer
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
        flywheel.setHoodGoalPos(1247);
        flywheel.update();
        telemetry.addData("pose", PedroComponent.follower().getPose());
        telemetry.update();
    }

    @Override
    public void onStartButtonPressed() {
        //turret.zeroTurret();
        autonomous.schedule();
    }
    @Override
    public void onUpdate() {
        turret.setCurrentPose(PedroComponent.follower().getPose(), PedroComponent.follower().getVelocity());
        turret.update();

        telemetry.addData("pose", PedroComponent.follower().getPose());
        flywheel.update();
        telemetry.update();
    }

    @Override
    public void onStop() {
        OpModeTransfer.currentPose = PedroComponent.follower().getPose();
        OpModeTransfer.alliance = Alliance.RED;
        OpModeTransfer.hasBeenTransferred = true;
    }
}
