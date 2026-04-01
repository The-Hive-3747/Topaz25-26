package org.firstinspires.ftc.teamcode.opmodes.auto;

import com.pedropathing.follower.Follower;
import com.pedropathing.geometry.Pose;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

public class AutoPaths {
    public static Pose startingPose, curveIntake2, closeShootingPose, intakeHPStartPose, intakeHPEndPose,
            farShootingPose, intake1StartPose, intake1EndPose, intake2StartPose, intake2EndPose,
            backParkPose, frontParkPose, openGateStartPose, openGateEndPose, intake3StartPose,
            intake3EndPose, farJigglePose, gateIntakeEndPose, gateIntakeStartPose;

    public static double closeShootAngle, shootAngle, parkAngle, startAngle, intakeAngle, openGateAngle, gateIntakeAngle;
    public static Alliance alliance;
    public static Follower follower;
    public static boolean customParkPose;

    /**
     * Generates all poses. Must be called before building paths.
     * Call AFTER setting alliance, starting pose, etc.
     * @param follow PedroPathing follower
     */
    public static void generatePoses(Follower follow) {
        follower = follow;

        // DEFINE ANGLES
        if (!customParkPose) {
            parkAngle = flipHeading180Degrees(180);
        }
        closeShootAngle = flipHeading180Degrees(180);
        shootAngle = flipHeading180Degrees(180);
        intakeAngle = flipHeading180Degrees(180);
        openGateAngle = Math.toRadians(90);
        if (alliance == Alliance.BLUE) {
            gateIntakeAngle = Math.toRadians(165);
        } else {
            gateIntakeAngle = Math.toRadians(15);
        }

        // DEFINE POSES
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
        gateIntakeStartPose = flipOverCenter(new Pose(30, 55, gateIntakeAngle));
        gateIntakeEndPose = flipOverCenter(new Pose(17, 59.2, gateIntakeAngle));
        backParkPose = flipOverCenter(new Pose(36.25, 20.5, parkAngle));
        closeShootingPose = flipOverCenter(new Pose(54.25, 88.75, shootAngle));
        farShootingPose = flipOverCenter(new Pose(55, 21, shootAngle));
        intakeHPStartPose = flipOverCenter(new Pose(28.5, 10.5, intakeAngle));
        intakeHPEndPose = flipOverCenter(new Pose(10.5, 10.5, intakeAngle));
        intake1StartPose = flipOverCenter(new Pose(51.25, 80.25, intakeAngle));
        intake1EndPose = flipOverCenter(new Pose(27.5, 80.25, intakeAngle));
        openGateStartPose = flipOverCenter(new Pose(35, 76, intakeAngle));
        openGateEndPose = flipOverCenter(new Pose(18.5, 76, intakeAngle));
        intake2StartPose = flipOverCenter(new Pose(56.25, 57.75, intakeAngle));
        intake2EndPose = flipOverCenter(new Pose(18.5, 57.75, intakeAngle));
        intake3StartPose = flipOverCenter(new Pose(56.25, 33.75, intakeAngle));
        intake3EndPose = flipOverCenter(new Pose(20, 33.75, intakeAngle));
        curveIntake2 = flipOverCenter(new Pose(50, 72));
        farJigglePose = flipOverCenter(new Pose(55, 17, shootAngle));
    }

    /**
     * Flips a Pose over the center line for red alliance.
     * @param pose your start pose (MUST HAVE HEADING)
     * @return Pose which has been flipped
     */
    private static Pose flipOverCenter(Pose pose) {
        if (alliance == Alliance.BLUE) {
            return pose;
        }
        double newPoseX = 144 - pose.getX();
        return new Pose(newPoseX, pose.getY(), pose.getHeading());
    }

    /**
     * @param heading in degrees
     * @return heading in radians, flipped 180 degrees for red alliance
     */
    private static double flipHeading180Degrees(double heading) {
        if (alliance == Alliance.BLUE) {
            return Math.toRadians(heading);
        }
        return Math.toRadians((heading + 180) % 360);
    }

    /**
     * Must be called before generatePoses()
     * @param pose start pose, MUST HAVE HEADING
     */
    public static void setStartPose(Pose pose) {
        startingPose = pose;
    }

    /**
     * Must be called before generatePoses()
     * @param pose park pose
     */
    public static void setFrontParkPose(Pose pose) {
        frontParkPose = pose;
    }

    /**
     * Must be called before generatePoses()
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
}