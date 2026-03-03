package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.geometry.Pose;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

@Autonomous(name = "front blue auto")
public class FrontBlueAuto extends AutoTemplate {
    @Override
    public void initAuto() {
        super.startPose = new Pose(63.25, 7.585,90);
        super.alliance = Alliance.BLUE;


        // if we do anything custom with paths, we should override this:
        /*
        autonomousCommands = new SequentialGroup(
                new ParallelGroup(
                        startAimbotFlywheel,
                        new FollowPath(toShootFromStart),
                        intake.railDownAuto
                ),
                new Delay(1.5),
                new ParallelGroup(
                        intake.shootAllThree
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake1),
                        intake.startIntake
                ),
                new ParallelGroup(
                        intake.startIntake,
                        new FollowPath(intake1)
                ),
                new Delay(1),
                new FollowPath(toShootFromIntake1),
                intake.stopIntake,
                new ParallelGroup(
                        intake.shootAllThree
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake2),
                        intake.startIntake
                ),
                new FollowPath(intake2),
                new Delay(1),
                intake.stopIntake,
                new FollowPath(toShootFromIntake2),
                new Delay(0.3),
                new ParallelGroup(
                        intake.shootAllThree
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        new FollowPath(lineUpForIntake3),
                        intake.startIntake
                ),
                new FollowPath(intake3),
                new Delay(1.3),
                intake.stopIntake,
                new FollowPath(toShootFromIntake3),
                new Delay(0.3),
                new ParallelGroup(
                        intake.shootAllThree
                ),
                new ParallelGroup(
                        intake.firewheelsOff,
                        flywheel.stopFlywheel,
                        new FollowPath(park)
                )
        );
        */
    }

}