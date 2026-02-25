package org.firstinspires.ftc.teamcode.subsystems;

import dev.nextftc.core.components.Component;
import dev.nextftc.core.units.Distance;
import dev.nextftc.ftc.ActiveOpMode;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.InvertedFTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.qualcomm.hardware.limelightvision.Limelight3A;
import com.qualcomm.hardware.limelightvision.LLResult;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.Pose3D;

public class Relocalization implements Component{
    //1 meter = 39.37 inches
    double METERS_TO_INCHES = 39.37;
    Limelight3A limelight;
    int maxFreshness = 20;
    LLResult result;
    Pose botPosePedro;
    public double x = 0;
    public double y = 0;
    public double heading = 0;



@Override
    public void preInit() {
    limelight = ActiveOpMode.hardwareMap().get(Limelight3A.class, "limelight");
    //Only uncomment this if working on limelight
    limelight.start();
    //limelight.shutdown();


}

    public boolean isDataFresh(){
        return result != null && result.isValid() && result.getStaleness() < maxFreshness;
    }

    public Pose getPedroPose(){
        return botPosePedro.getPose();
    }



    public void update() {
    result = limelight.getLatestResult();
    if (result != null && result.isValid()) {
        Pose3D botPose = result.getBotpose();
        if (botPose != null) {
            // 72 is what we have to add in order to get to Pedro Coordinates manually
            x = botPose.getPosition().y * METERS_TO_INCHES + 72;
            y = -botPose.getPosition().x * METERS_TO_INCHES + 72;
            // 90 is what we have to use in order to convert the heading to Pedro Coordinates manually
            heading = Math.toRadians(botPose.getOrientation().getYaw() - 90);
            botPosePedro = new Pose(x, y, heading);

            ActiveOpMode.telemetry().addData("--------------------I SEE TAG-----------------------", "");
            ActiveOpMode.telemetry().addData("Bot X", x);
            ActiveOpMode.telemetry().addData("Bot Y", y);
            ActiveOpMode.telemetry().addData("Bot Heading", Math.toDegrees(heading));
            ActiveOpMode.telemetry().addData("Bot pos", botPose.getPosition().toUnit(DistanceUnit.INCH));

            //Convert the Pose3D to a Pose2D in order to convert to Pedro and multiply by METERS_TO_INCHES to convert the values from meters to inches
            /*Pose2D botPose2D = new Pose2D(DistanceUnit.INCH ,botPose.getPosition().x * METERS_TO_INCHES, botPose.getPosition().y * METERS_TO_INCHES, AngleUnit.RADIANS ,botPose.getOrientation().getYaw());
            //Convert the Pose2D into Pedro Pose
            botPosePedro = PoseConverter.pose2DToPose(botPose2D, FTCCoordinates.INSTANCE);
            //Make the coordinate system to Pedro's coordinate system.
            botPosePedro.getAsCoordinateSystem(PedroCoordinates.INSTANCE);
            x = botPosePedro.getX();
            y = botPosePedro.getY();
            heading = botPosePedro.getHeading();*/
        }
    }
}



}
