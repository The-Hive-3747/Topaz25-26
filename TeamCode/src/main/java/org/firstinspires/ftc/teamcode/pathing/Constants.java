package org.firstinspires.ftc.teamcode.pathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
import com.pedropathing.control.PredictiveBrakingCoefficients;
import com.pedropathing.follower.Follower;
import com.pedropathing.follower.FollowerConstants;
import com.pedropathing.ftc.FollowerBuilder;
import com.pedropathing.ftc.drivetrains.MecanumConstants;
import com.pedropathing.ftc.localization.constants.PinpointConstants;
import com.pedropathing.paths.PathConstraints;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

public class Constants {
    public static FollowerConstants followerConstants = new FollowerConstants()
            .mass(13.6)//14.1//13.8// 11.8// 13.7//V2.5 robot mass //V2 robot mass = 12.65
            .useSecondaryHeadingPIDF(true)
            .headingPIDFCoefficients(new PIDFCoefficients(0.8, 0,0.05,0.02))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(1.5,0,0.01,0))
            .predictiveBrakingCoefficients(new PredictiveBrakingCoefficients(0.3, 0.083,0.0017))
            .centripetalScaling(0);




    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1.0)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(76.38)//(75.89)//73.32//75.10//57.49//61.78
            .yVelocity(58.3);//(59.51);//57.25//58.18//45.39//49.90 //(24.53);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-6.64)//-7//(5.4)//(0)
            .strafePodX(-1.48)//-0.125//(0)//(-5.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // defaults: (0.995, 0.1, 0.1, 0.007, 100, 1, 10, 1);
    public static PathConstraints pathConstraints = new PathConstraints(0.9, 0.3, 0.3, 0.2, 50, 1.2, 10, 1);//(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}