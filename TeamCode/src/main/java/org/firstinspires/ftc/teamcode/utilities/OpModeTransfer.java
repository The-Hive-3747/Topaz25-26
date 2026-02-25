package org.firstinspires.ftc.teamcode.utilities;

import com.pedropathing.geometry.Pose;

public class OpModeTransfer {
    public static Pose currentPose = new Pose(72,72,Math.toRadians(90));
    public static Pose resetRedPose = new Pose(121,123,Math.toRadians(37));
    public static Pose resetBluePose = new Pose(22,124,Math.toRadians(138));
    public static boolean hasBeenTransferred = false;
    public static Alliance alliance = Alliance.RED;
}
