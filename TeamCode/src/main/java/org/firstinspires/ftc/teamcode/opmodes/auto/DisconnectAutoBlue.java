package org.firstinspires.ftc.teamcode.opmodes.auto;

import static org.firstinspires.ftc.teamcode.opmodes.auto.AutoPaths.*;

import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.Path;
import com.pedropathing.paths.PathChain;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

import dev.nextftc.core.commands.delays.Delay;
import dev.nextftc.core.commands.groups.ParallelGroup;
import dev.nextftc.core.commands.groups.SequentialGroup;
import dev.nextftc.extensions.pedro.FollowPath;

@Autonomous(name = "disconnect auto blue")
public class DisconnectAutoBlue extends AutoTemplate {
    Pose disconnectShootPose = new Pose(71.8,20.5, Math.toRadians(51.7));
    Pose humanIntakePose = new Pose(127.7 ,11.4, Math.toRadians(-3.3));
    Pose parkPose = new Pose(71.8, 32.5, Math.toRadians(180));
    @Override
    public void initAuto() {
        super.alliance = Alliance.BLUE;
        super.startPose = new Pose(63.25, 7.585,Math.toRadians(90));
        super.parkAngle = Math.toRadians(0);
        super.parkPose = new Pose(36.25, 78.25, parkAngle);

        PathChain startToBackShoot = generatePath(startPose, disconnectShootPose);
        PathChain backShootToHuman = generatePath(disconnectShootPose, humanIntakePose);
        PathChain humanToBackShoot = generatePath(humanIntakePose, disconnectShootPose);
        PathChain backShootToPark = generatePath(disconnectShootPose, parkPose);
        super.autonomousCommands = new SequentialGroup(
                new ParallelGroup(
                        startAimbotFlywheel,
                        turret.setTurretForAuto,
                        new FollowPath(startToBackShoot),
                        intake.railDownAuto
                ),
                new Delay(1.5),
                new ParallelGroup(
                        intake.shootAllThree
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        new FollowPath(backShootToHuman),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake
                        //new FollowPath(intake1)
                ),
                new Delay(1),
                new FollowPath(humanToBackShoot),
                intake.stopIntake,
                new ParallelGroup(
                        intake.shootAllThree
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        flywheel.stopFlywheel,
                        new FollowPath(backShootToPark),
                        turret.setTurretForTeleop
                )
        );
    }

}