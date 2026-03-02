package org.firstinspires.ftc.teamcode.opmodes;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

public class AutoPaths {
    public static Pose startingPose, shootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, parkPose, toShootCurvePose, openGateStartPose, openGateEndPose, intake3StartPose, intake3EndPose, lastShootingPose;
    public static PathChain toShootFromStart, lineUpForIntake1, intake1, lineUpForOpenGate, toShootFromIntake1, lineUpForIntake2, intake2, toShootFromIntake2, park, openGate, toShootFromOpenGate, lineUpForIntake3, intake3, toShootFromIntake3;
    public static double shootAngle, parkAngle, startAngle, intakeAngle, lastShootAngle;
    public static Alliance alliance;
    public static Follower follower;

    /**
     * this generates all poses & paths. must be called before start of auto.
     * this should be called AFTER you've set the alliance, starting pose, etc
     * @param follow PedroPathing follower
     */
    public static void generatePaths(Follower follow) {
        follower = follow;

        if (startingPose == null) {
            if (alliance == Alliance.BLUE) {
                startingPose = new Pose(34.5, 135.8, Math.toRadians(-85.05));
            } else {
                startingPose = new Pose(109.5, 135.8, Math.toRadians(-94.95));
            }
        }

        // DEFINE POSES HERE
        shootingPose = new Pose(54.25, 88.75, Math.toRadians(40));
        intake1StartPose = new Pose(51.25, 79.75); //y:81 34//y:82//x: 47 y:78
        intake1EndPose = new Pose(24.25, 79.75); //6//x:16 y:82//x: 16 :78
        openGateStartPose = new Pose(22, 74); //78//x:35
        openGateEndPose = new Pose(14, 74);//x:18
        intake2StartPose = new Pose(51.25, 57.75);//y:58//y: 61
        intake2EndPose = new Pose(14.25, 57.75);//x:15 x:8 y:58//x: 9 y:61
        intake3StartPose = new Pose(56.25, 33.75);//y:38//y: 32
        intake3EndPose = new Pose(14.25, 33.75);//x:8 y:38//y: 32
        parkPose = new Pose(36.25, 78.25);
        toShootCurvePose = new Pose(86.25,70.75);
        lastShootingPose = new Pose(50, 106);

        // GENERATE PATHS HERE
        toShootFromStart = generatePath(startingPose, shootingPose);
        lineUpForIntake1 = generatePath(shootingPose, intake1StartPose);
        intake1 = generatePath(intake1StartPose, intake1EndPose);
        lineUpForOpenGate = generatePath(intake1EndPose, openGateStartPose);
    }


    /**
     * Flips a Pose over the center line.
     *
     * @param pose your start pose
     * @return Pose which has been flipped
     */

    private static Pose flipOverCenter(Pose pose) {
        if (alliance == Alliance.BLUE) {
            return pose;
        }

        // we subtract the x from 144 to flip the x
        // y stays the same for this game
        double newPoseX = 144-pose.getX();
        return new Pose(newPoseX, pose.getY());
    }

    /**
     *
     * @param heading in degrees
     * @return heading in radians, flipped 180 degrees
     */
    private static double flipHeading180Degrees(double heading) {
        return Math.toRadians(heading + 180);
    }

    /**
     * make sure to use this before you use generatePaths()
     * @param pose start pose, MUST HAVE HEADING
     */
    public static void setStartPose(Pose pose) {
        startingPose = pose;
    }

    /**
     * @return Alliance, either blue or red
     */
    public static Alliance getAlliance() {
        return alliance;
    }

    /**
     * generates a PathChain using the heading from pose1 & pose2
     * @param pose1 the first pose (MUST HAVE HEADING)
     * @param pose2 the second pose (MUST HAVE HEADING)
     * @return the built PathChain
     */
    public static PathChain generatePath(Pose pose1, Pose pose2) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
    }

    /**
     * generates a PathChain with a constant heading
     * @param pose1 the first pose
     * @param pose2 the second pose
     * @param heading IN RADIANS, the constant heading to be followed
     * @return the built PathChain
     */
    public static PathChain generatePath(Pose pose1, Pose pose2, double heading) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setConstantHeadingInterpolation(heading)
                .build();
    }

    /**
     * generates a PathChain using set headings and poses
     * @param pose1 the first pose
     * @param pose2 the second pose
     * @param heading1 IN RADIANS, the first heading to be followed
     * @param heading2 IN RADIANS, the second heading to be followed
     * @return the built PathChain
     */
    public static PathChain generatePath(Pose pose1, Pose pose2, double heading1, double heading2) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setLinearHeadingInterpolation(heading1, heading2)
                .build();
    }
}