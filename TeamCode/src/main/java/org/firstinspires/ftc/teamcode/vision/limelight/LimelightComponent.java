package org.firstinspires.ftc.teamcode.vision.limelight;
import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.core.components.Component;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;

import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;


public class LimelightComponent implements Component{
    private Limelight3A limelight;
    private boolean hasTarget = false;
    private double targetX = 0.0;
    private double targetY= 0.0;
    private double targetArea = 0.0;
    private double targetHeading = 0.0;
    private int aprilTagId = -1;
    private double robotX = 0.0;
    private double robotY = 0.0;
    private double robotHeading = 0.0;


    public void init() {
        limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "Limelight");
        limelight.start();
    }

    public void update(double currentHeadingDegrees) {
        limelight.updateRobotOrientation(currentHeadingDegrees);
        LLResult result = limelight.getLatestResult();

        if(result != null && result.isValid()) {
            hasTarget = true;
            targetX = result.getTx();
            targetY = result.getTy();
            targetArea = result.getTa();
            Pose3D botpose = result.getBotpose();
            if(botpose != null) {
                robotX = botpose.getPosition().x;
                robotY = botpose.getPosition().y;
                robotHeading = botpose.getOrientation().getYaw();
                targetHeading = robotHeading;
            }
            if(result.getFiducialResults() != null && !result.getFiducialResults().isEmpty()) {
                aprilTagId = (int) result.getFiducialResults().get(0).getFiducialId();
            } else {
                aprilTagId = -1;
            }
        } else {
            hasTarget = false;
            aprilTagId = -1;
        }
    }

    public double getRobotX() {
        return robotX;
    }
    public double getRobotY() {
        return robotY;
    }
    public double getRobotHeading() {
        return robotHeading;
    }

    public boolean hasTarget() {
        return hasTarget;
    }
    public double getTargetX() {
        return targetX;
    }
    public double getTargetY() {
        return targetY;
    }
    public double getTargetArea() {
        return targetArea;
    }

    public double getTargetHeading() {
        return targetHeading;
    }
    public int getAprilTagId() {
        return aprilTagId;
    }
    public boolean hasValidBotPose() {
        return hasTarget && (robotX!=0.0 || robotY!=0.0);
    }
}
