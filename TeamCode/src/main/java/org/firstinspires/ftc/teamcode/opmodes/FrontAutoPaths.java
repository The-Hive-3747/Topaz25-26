package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

public class FrontAutoPaths {
    public static Pose startingPose, shootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, parkPose, toShootCurvePose, openGateStartPose, openGateEndPose, intake3StartPose, intake3EndPose, lastShootingPose, moveAutoEnd;
    public static PathChain toShootFromStart, lineUpForIntake1, intake1, lineUpForOpenGate, toShootFromIntake1, lineUpForIntake2, intake2, toShootFromIntake2, park, openGate, toShootFromOpenGate, lineUpForIntake3, intake3, toShootFromIntake3, toEndFromStart;
    public static double shootAngle, parkAngle, startAngle, intakeAngle, lastShootAngle;
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
            startingPose = new Pose(34.5, 131.5); // X AND Y ARE UPDATED TO THE REAL WORLD
            startAngle = Math.toRadians(-84.7);
        } else {
            startingPose = new Pose(110.5, 131.5); // X AND Y ARE UPDATED TO THE REAL WORLD
            startAngle = Math.toRadians(-96.5);
        }
        shootingPose = convert(new Pose(54.25, 88.75));
        intake1StartPose = convert(new Pose(51.25, 79.75)); //y:81 34//y:82//x: 47 y:78
        intake1EndPose = convert(new Pose(24.25, 79.75)); //6//x:16 y:82//x: 16 :78
        openGateStartPose = convert(new Pose(22, 74)); //78//x:35
        openGateEndPose = convert(new Pose(14, 74));//x:18
        intake2StartPose = convert(new Pose(51.25, 57.75));//y:58//y: 61
        intake2EndPose = convert(new Pose(14.25, 57.75));//x:15 x:8 y:58//x: 9 y:61
        intake3StartPose = convert(new Pose(56.25, 33.75));//y:38//y: 32
        intake3EndPose = convert(new Pose(14.25, 33.75));//x:8 y:38//y: 32
        parkPose = convert(new Pose(36.25, 78.25));
        toShootCurvePose = convert(new Pose(86.25,70.75));
        lastShootingPose = convert(new Pose(50, 106));
        moveAutoEnd = convert(new Pose( 10, 8.62));

        if (alliance == Alliance.RED) {
            shootAngle = convertHeading90(Math.toRadians(40));
            parkAngle = convertHeading180(Math.toRadians(180));
            intakeAngle = convertHeading180(Math.toRadians(180));
            lastShootAngle = convertHeading90(Math.toRadians(0));
        } else {
            shootAngle = Math.toRadians(230);
            parkAngle = convertHeading180(Math.toRadians(180));
            intakeAngle = convertHeading180(Math.toRadians(180));
            lastShootAngle = Math.toRadians(-90);
        }
        //shootAngle = convertHeading90(Math.toRadians(40));//135//210//180//0//90//110


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
                .setLinearHeadingInterpolation(shootAngle, intakeAngle)
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
                .setLinearHeadingInterpolation(intakeAngle, Math.toRadians(-90))
                .setVelocityConstraint(0.5)
                .build();

        openGate = follower
                .pathBuilder()
                .addPath(new BezierLine(openGateStartPose, openGateEndPose))
                .setConstantHeadingInterpolation(Math.toRadians(-90))
                .setVelocityConstraint(0.5)
                .build();

        toShootFromOpenGate = follower
                .pathBuilder()
                .addPath(new BezierCurve(openGateEndPose, toShootCurvePose, shootingPose))
                .setLinearHeadingInterpolation(Math.toRadians(-90), shootAngle)
                .build();

        toShootFromIntake1 = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(intake1EndPose, shootingPose)
                )
                .setLinearHeadingInterpolation(intakeAngle, shootAngle)
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
                .setVelocityConstraint(0.75)
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
        toEndFromStart = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(startingPose, moveAutoEnd)
                )
                .setConstantHeadingInterpolation(startAngle)
                .build();

        park = follower
                .pathBuilder()
                .addPath(
                        new BezierLine(shootingPose, parkPose)
                )
                .setLinearHeadingInterpolation(shootAngle, parkAngle)
                .build();
    }
}
