package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.BezierCurve;
import com.pedropathing.geometry.BezierLine;
import com.pedropathing.geometry.Pose;
import com.pedropathing.paths.PathChain;
import com.pedropathing.paths.PathConstraints;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

public class AutoPaths {
    public static Pose intakeGateStartPose, farShootingPoseJiggle, closeShootingPoseJiggle, intakeGateEndPose, startingPose, curveIntake2,closeShootingPose, intakeHPStartPose, intakeHPEndPose, farShootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose, backParkPose, frontParkPose, intakeRecycledCloseStartPose, intakeRecycledCloseEndPose, openGateStartPose, openGateEndPose, intake3StartPose, intake3EndPose, farJigglePose, intakeRecycledFarStartPose, intakeRecycledFarEndPose;
    public static PathChain intakeGate, lineUpForIntakeGate, shootCloseJiggle, toShootAtCloseFromLastPoseCurved, lineUpForIntakeHPFromLastPose, intakeHP, lineUpForIntake1FromLastPose, intake1, lineUpForOpenGateFromLastPose, lineUpForIntake2FromLastPose, intake2, parkAtBackFromLastPose, parkAtFrontFromLastPose, openGate, lineUpForIntakeRecycledClose, intakeRecycledClose, lineUpForIntake3FromLastPose, intake3, toShootAtCloseFromLastPose, toShootAtFarFromLastPose, shootFarJiggle, lineUpForIntakeRecycledFar, intakeRecycledFar;
    public static double intakeHPAngle, gateIntakeAngle, closeShootAngle, shootAngle, parkAngle, startAngle, intakeAngle, openGateAngle;
    public static Alliance alliance;
    public static Follower follower;
    public static boolean customParkPose;

    /**
     * this generates all poses. must be called before start of auto.
     * this should be called AFTER you've set the alliance, starting pose, etc
     * @param follow PedroPathing follower
     */
    public static void generatePoses(Follower follow) {
        follower = follow;

        // DEFINE ANGLES HERE
        if (!customParkPose) {
            parkAngle = flipHeading180Degrees(180);
        }
        if (alliance == Alliance.BLUE) {
            gateIntakeAngle = Math.toRadians(165);
            intakeHPAngle = Math.toRadians(-160);
        } else {
            gateIntakeAngle = Math.toRadians(15);
            intakeHPAngle = Math.toRadians(-20);
        }
        closeShootAngle = flipHeading180Degrees(180);
        shootAngle = flipHeading180Degrees(180);
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
        if (!customParkPose) {
            frontParkPose = flipOverCenter(new Pose(36.25, 78.25, parkAngle));
        }
        backParkPose = flipOverCenter(new Pose(36.25, 20.5, parkAngle));

        closeShootingPose = flipOverCenter(new Pose(51.25, 87.75, shootAngle));//x:54.25 y:88.75
        closeShootingPoseJiggle = flipOverCenter(new Pose(51, 87.5, shootAngle));//x:54 y:88.5

        farShootingPose = flipOverCenter(new Pose(50.9, 17.9, shootAngle));//x:54.1 y:18.1
        farShootingPoseJiggle = flipOverCenter(new Pose(51, 18, shootAngle));//x:54 y:18
        intakeHPStartPose = flipOverCenter(new Pose(47, 15, intakeHPAngle)); //x:50 y:15 //intakeHpAngle
        intakeHPEndPose = flipOverCenter(new Pose(8,10.5, intakeAngle));//x:10, y:10.5 //16 //10
        intakeRecycledFarStartPose = flipOverCenter(new Pose(47, 15, intakeHPAngle)); //x:50 y:15
        intakeRecycledFarEndPose = flipOverCenter(new Pose(9,11.5, intakeAngle));//x:16, y:11.5 //15
        intake1StartPose = flipOverCenter(new Pose(48.25, 82.25, intakeAngle));//x:51.25 y:81.25//y:81 34//y:82//x: 47 y:78
        intake1EndPose = flipOverCenter(new Pose(14, 82.25, intakeAngle)); //x:19 y:81.25//6//x:16 y:82//x: 16 :78
        openGateStartPose = flipOverCenter(new Pose(32, 77, intakeAngle)); //x:35 y:76//78//x:35
        openGateEndPose = flipOverCenter(new Pose(12, 77, intakeAngle));//x:13 y:76//x:18.5
        intake2StartPose = flipOverCenter(new Pose(53.25, 56.75, intakeAngle));//x:56.25 y:57.75//y:58//y: 61
        intake2EndPose = flipOverCenter(new Pose(3.5, 56.75, intakeAngle));//x:11.5 y:57.75//x:15 x:8 y:58//x: 9 y:61
        intakeRecycledCloseStartPose = flipOverCenter(new Pose(53.25, 42.75, intakeAngle));//x:56.25 y:57.75//y:58//y: 61
        intakeRecycledCloseEndPose = flipOverCenter(new Pose(3.5, 42.75, intakeAngle));//x:11.5 y:57.75//x:15 x:8 y:58//x: 9 y:61
        intake3StartPose = flipOverCenter(new Pose(53.25, 34.75, intakeAngle));//x:56.25 y:33.75//y:38//y: 32
        intake3EndPose = flipOverCenter(new Pose(8, 34.75, intakeAngle));//x:16, y:33.75//x:11.5 y:38//y: 32
        curveIntake2 = flipOverCenter(new Pose(55,68));
        farJigglePose = flipOverCenter(new Pose (55,17, shootAngle));


        intakeGateStartPose = flipOverCenter(new Pose(30, 55, gateIntakeAngle));
        intakeGateEndPose = flipOverCenter(new Pose(17, 59.2, gateIntakeAngle));
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
    public static void setFrontParkPose(Pose pose) {
        frontParkPose = pose;
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
        return generatePath(pose1, pose2, pose1.getHeading(), pose2.getHeading());
    }



    /**
     * generates a PathChain using the heading from pose1 & pose2
     * @param pose1 the first pose (MUST HAVE HEADING)
     * @param pose2 the second pose (MUST HAVE HEADING)
     * @return the built PathChain
     */
    public static PathChain generatePathShortCallback(Pose pose1, Pose pose2) {
        if (pose1.getHeading() == pose2.getHeading()) {
            return follower.pathBuilder()
                    .addPath(
                            new BezierLine(pose1, pose2)
                    )
                    .setConstantHeadingInterpolation(pose1.getHeading())
                    .setConstraints(new PathConstraints(0.6, 100))
                    .build();
        }
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .setConstraints(new PathConstraints(0.6, 100))
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
        if (heading1 == heading2) {
            return generatePath(pose1, pose2, heading1);
        }
        return follower.pathBuilder()
                .addPath(
                        new BezierLine(pose1, pose2)
                )
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
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
     * generates a PathChain with a linear heading from pose1 & pose2
     * this is a bezier curve
     * @param pose1 the first pose
     * @param curvePose the curve pose
     * @param pose2 the second pose
     * @param velocityConstraint
     * @return the built PathChain
     */
    public static PathChain generatePathCurveWithVelocityConstraint(Pose pose1, Pose curvePose, Pose pose2, double velocityConstraint) {
        return follower.pathBuilder()
                .addPath(
                        new BezierCurve(pose1, curvePose, pose2)
                )
                .setLinearHeadingInterpolation(pose1.getHeading(), pose2.getHeading())
                .setVelocityConstraint(velocityConstraint)
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