package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

public class FrontAutoPathsOld {
    public static Pose startingPose, shootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, parkPose, toShootCurvePose, openGateStartPose, openGateEndPose, intake3StartPose, intake3EndPose;
    public static PathChain toShootFromStart, lineUpForIntake1, intake1, lineUpForOpenGate, lineUpForIntake2, intake2, toShootFromIntake2, park, openGate, toShootFromOpenGate, lineUpForIntake3, intake3, toShootFromIntake3;
    public static double shootAngle, parkAngle, startAngle, intakeAngle;
    public static Alliance alliance;
    private static Pose convert(Pose pose) {
        if (alliance == Alliance.BLUE) {
            return pose;
        }

        double newPoseX = 144-pose.getX();
        return new Pose(newPoseX, pose.getY());
    }

    private static double convertHeading180(double heading) {
        if (alliance == Alliance.BLUE) {
            return heading;
        }
        return heading - Math.PI;
    }
    private static double convertHeading90(double heading) {
        if (alliance == Alliance.BLUE) {
            return heading;
        }
        return heading - Math.PI/2;
    }

    public static Alliance getAlliance() {
        return alliance;
    }


    public static void generatePaths(Follower follower) {
        if (alliance == Alliance.BLUE) {
            startingPose = new Pose(21, 120);
        } else {
            startingPose = new Pose(125, 122);
        }
        shootingPose = convert(new Pose(48, 90));
        intake1StartPose = convert(new Pose(45, 82)); //34
        intake1EndPose = convert(new Pose(16, 82)); //6
        openGateStartPose = convert(new Pose(35, 74)); //78
        openGateEndPose = convert(new Pose(18, 74));
        intake2StartPose = convert(new Pose(45, 58));
        intake2EndPose = convert(new Pose(8, 58));
        intake3StartPose = convert(new Pose(50, 38));
        intake3EndPose = convert(new Pose(8, 38));
        parkPose = convert(new Pose(30, 80));
        toShootCurvePose = convert(new Pose(80,72));

        shootAngle = convertHeading90(Math.toRadians(135));
        parkAngle = convertHeading180(Math.toRadians(180));
        if (alliance == Alliance.BLUE) {
            startAngle = Math.toRadians(143);
        } else {
            startAngle = Math.toRadians(39);
        }
        intakeAngle = convertHeading180(Math.toRadians(180));

        toShootFromStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startingPose, shootingPose)
                )
                .setLinearHeadingInterpolation(startAngle, shootAngle)
                .build();

        lineUpForIntake1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, intake1StartPose)
                )
                .setLinearHeadingInterpolation(startAngle, intakeAngle)
                .build();

        intake1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake1StartPose, intake1EndPose)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        lineUpForOpenGate = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(intake1EndPose, openGateStartPose)
                )
                .setLinearHeadingInterpolation(intakeAngle, Math.toRadians(90))
                .setVelocityConstraint(0.5)
                .build();

        openGate = follower
                .pathBuilder()
                .addPath(new BezierLine(openGateStartPose, openGateEndPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .setVelocityConstraint(0.5)
                .build();

        toShootFromOpenGate = follower
                .pathBuilder()
                .addPath(new BezierCurve(openGateEndPose, toShootCurvePose, shootingPose))
                .setConstantHeadingInterpolation(Math.toRadians(90))
                .build();

        lineUpForIntake2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, intake2StartPose)
                )
                .setLinearHeadingInterpolation(shootAngle, intakeAngle)
                .build();

        intake2 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake2StartPose, intake2EndPose)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        toShootFromIntake2 = follower
                .pathBuilder()
                .addPath(
                        new BezierCurve(intake2EndPose, toShootCurvePose, shootingPose)
                )
                .setLinearHeadingInterpolation(intakeAngle, shootAngle)
                .build();

        lineUpForIntake3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, intake3StartPose)
                )
                .setLinearHeadingInterpolation(shootAngle, intakeAngle)
                .build();

        intake3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake3StartPose, intake3EndPose)
                )
                .setConstantHeadingInterpolation(intakeAngle)
                .build();

        toShootFromIntake3 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake3EndPose, shootingPose)
                )
                .setLinearHeadingInterpolation(intakeAngle, shootAngle)
                .build();


        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, parkPose)
                )
                .setLinearHeadingInterpolation(shootAngle, parkAngle)
                .build();
    }

    public PathChain getToShootFromStart() { return toShootFromStart; }
    public PathChain getLineUpForIntake1() { return lineUpForIntake1; }

    public static PathChain getIntake1() {
        return intake1;
    }

    public static PathChain getLineUpForIntake2() {
        return lineUpForIntake2;
    }

    public static PathChain getIntake2() {
        return intake2;
    }

    public static PathChain getOpenGate() {
        return openGate;
    }

    public static PathChain getToShootFromIntake2() {
        return toShootFromIntake2;
    }

    public static PathChain getPark() {
        return park;
    }
}
