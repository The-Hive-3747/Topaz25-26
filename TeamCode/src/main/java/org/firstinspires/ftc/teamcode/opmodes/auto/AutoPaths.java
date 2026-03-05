package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

public class AutoPaths {
    public static Pose startingPose, frontShootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, parkPose, toShootCurvePose, openGateStartPose, openGateEndPose, intake3StartPose, intake3EndPose, lastShootingPose;
    public static PathChain toShootFromStart, lineUpForIntake1, intake1, lineUpForOpenGate, toShootFromIntake1, lineUpForIntake2, intake2, toShootFromIntake2, park, openGate, toShootFromOpenGate, lineUpForIntake3, intake3, toShootFromIntake3, toShootAtFrontFromLastPose;
    public static double closeShootAngle, parkAngle, startAngle, intakeAngle, lastShootAngle, openGateAngle;
    public static Alliance alliance;
    public static Follower follower;

    /**
     * this generates all poses & paths. must be called before start of auto.
     * this should be called AFTER you've set the alliance, starting pose, etc
     * @param follow PedroPathing follower
     */
    public static void generatePaths(Follower follow) {
        follower = follow;

        // DEFINE ANGLES HERE
        if (parkAngle == 0.0d) { //checking for null with 0.0d because doubles cant be null
            parkAngle = flipHeading180Degrees(180);
        }
        if (alliance == Alliance.BLUE) {
            closeShootAngle = Math.toRadians(135); //convertHeading90(Math.toRadians(40));
        } else {
            closeShootAngle = Math.toRadians(50); //convertHeading90(Math.toRadians(40));
        }
        intakeAngle = flipHeading180Degrees(180);
        openGateAngle = Math.toRadians(90);

        // DEFINE POSES HERE
        if (startingPose == null) {
            if (alliance == Alliance.BLUE) {
                startingPose = new Pose(34.5, 135.8, Math.toRadians(-85.05));
            } else {
                startingPose = new Pose(109.5, 135.8, Math.toRadians(-94.95));
            }
        }

        if (parkPose == null) {
            parkPose = flipOverCenter(new Pose(36.25, 78.25, parkAngle));
        }
        frontShootingPose = flipOverCenter(new Pose(54.25, 88.75, closeShootAngle));
        intake1StartPose = flipOverCenter(new Pose(51.25, 79.75, intakeAngle)); //y:81 34//y:82//x: 47 y:78
        intake1EndPose = flipOverCenter(new Pose(27.25, 79.75, intakeAngle)); //6//x:16 y:82//x: 16 :78
        openGateStartPose = flipOverCenter(new Pose(22, 74, openGateAngle)); //78//x:35
        openGateEndPose = flipOverCenter(new Pose(14, 74, openGateAngle));//x:18
        intake2StartPose = flipOverCenter(new Pose(53, 57.75, intakeAngle));//y:58//y: 61
        intake2EndPose = flipOverCenter(new Pose(17.25, 57.75, intakeAngle));//x:15 x:8 y:58//x: 9 y:61
        intake3StartPose = flipOverCenter(new Pose(56.25, 33.75, intakeAngle));//y:38//y: 32
        intake3EndPose = flipOverCenter(new Pose(17.25, 33.75, intakeAngle));//x:8 y:38//y: 32
        parkPose = flipOverCenter(new Pose(36.25, 78.25, parkAngle));
        toShootCurvePose = flipOverCenter(new Pose(86.25,70.75));

        // GENERATE PATHS HERE
        toShootFromStart = generatePath(startingPose, frontShootingPose);

        lineUpForIntake1 = generatePath(frontShootingPose, intake1StartPose);
        intake1 = generatePath(intake1StartPose, intake1EndPose);
        toShootFromIntake1 = generatePath(intake1EndPose, frontShootingPose);

        lineUpForOpenGate = generatePath(intake1EndPose, openGateStartPose);
        openGate = generatePath(openGateStartPose, openGateEndPose);
        toShootFromOpenGate = generatePathCurve(openGateEndPose, toShootCurvePose, frontShootingPose);

        lineUpForIntake2 = generatePath(frontShootingPose, intake2StartPose);
        intake2 = generatePath(intake2StartPose, intake2EndPose);
        toShootFromIntake2 = generatePath(intake2EndPose, frontShootingPose);

        lineUpForIntake3 = generatePath(frontShootingPose, intake3StartPose);
        intake3 = generatePath(intake3StartPose, intake3EndPose);
        toShootFromIntake3 = generatePath(intake3EndPose, frontShootingPose);

        //toShootAtFrontFromLastPose = generatePath(AutoTemplate.lastPose, frontShootingPose);

        park = generatePath(frontShootingPose, parkPose);
    }


    /**
     * Flips a Pose over the center line.
     * MAKE SURE YOUR POSE HAS A HEADING. THE HEADING WILL BE KEPT THE SAME
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

        return new Pose(newPoseX, pose.getY(), pose.getHeading());
    }

    /**
     * @param heading in degrees
     * @return heading in radians, flipped 180 degrees
     */
    private static double flipHeading180Degrees(double heading) {
        if (alliance == Alliance.BLUE) {
            return Math.toRadians(heading);
        }
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
     * make sure to use this before you use generatePaths()
     * @param pose park pose, heading not required
     */
    public static void setParkPose(Pose pose) {
        parkPose = pose;
    }

    /**
     * make sure to use this before you use generatePaths()
     * @param heading park angle, RADIANS
     */
    public static void setParkAngle(double heading) {
        parkAngle = heading;
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

    /**
     * generates a PathChain with a linear heading from pose1 & pose2
     * this is a bezier curve
     * @param pose1 the first pose
     * @param curvePose the curve pose
     * @param pose2 the second pose
     * @return the built PathChain
     */
    public static PathChain generatePathCurve(Pose pose1, Pose curvePose, Pose pose2) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(pose1, curvePose, pose2)
                )
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .build();
    }

    /**
     * generates a PathChain with a linear heading from pose1 & pose2 and a velocity constraint
     * @param pose1 the first pose
     * @param pose2 the second pose
     * @param velocityConstraint double from 0-1
     * @return the built PathChain
     */
    public static PathChain generatePathWithVelocityConstraint(Pose pose1, Pose pose2, double velocityConstraint) {
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .setVelocityConstraint(velocityConstraint)
                .build();
    }
}