package org.firstinspires.ftc.teamcode.pathing;

import com.pedropathing.control.FilteredPIDFCoefficients;
import com.pedropathing.control.PIDFCoefficients;
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
            .mass(14.1)//13.8// 11.8// 13.7//V2.5 robot mass //V2 robot mass = 12.65
            .useSecondaryTranslationalPIDF(true)
            .useSecondaryHeadingPIDF(true)
            .useSecondaryDrivePIDF(true)
            .lateralZeroPowerAcceleration(-67.76)//-32.41//-77.91//-68.71 //-37.32) //-87.57
            .forwardZeroPowerAcceleration(-28.28)//-66.10//-45.59//-48.13 //-52.34
            .translationalPIDFCoefficients(new PIDFCoefficients(.75,0.0,0.1,0.001))//(0.53, 0, 0.08, 0))//(0.55, 0, 0.005, 0)//(0.03, 0, 0, 0))
            .secondaryTranslationalPIDFCoefficients(new PIDFCoefficients(0.15,0,0.05,0))//(0.2, 0, 0.012, 0))//(0.2, 0, 0.001, 0)//(0.1, 0, 0.01, 0))//just a guess
            .drivePIDFCoefficients(new FilteredPIDFCoefficients(0.1, 0, 0.0018, 0.6, 0.07))//(0.5,0,0.000,0.6,0.015))//(0.09, 0, 0.0001, 0.0, 0.00))
            .secondaryDrivePIDFCoefficients(new FilteredPIDFCoefficients(0.001, 0, 0.00005, 0.6, 0.0001))//(0.09, 0, 0.000005, 0.6, 0.01))//(0.001, 0, 0.00001, 0, 0.01))//just a guess
            .centripetalScaling(0.0005)//(0.001)
            .headingPIDFCoefficients(new PIDFCoefficients(2.0,0,0.155,0.01))//(0.9, 0, 0, 0.01))//(1, 0, 0, 0.01))
            .secondaryHeadingPIDFCoefficients(new PIDFCoefficients(3.2,0,0.3,0))//(4, 0, 0.15, -0.0008))//(0.25, 0, 0.01, -0.0008)//(0.1, 0, 0.01, 0))//just a guess
            .drivePIDFSwitch(5);//5



    public static MecanumConstants driveConstants = new MecanumConstants()
            .maxPower(1)
            .rightFrontMotorName("frontRightMotor")
            .rightRearMotorName("backRightMotor")
            .leftRearMotorName("backLeftMotor")
            .leftFrontMotorName("frontLeftMotor")
            .leftFrontMotorDirection(DcMotorSimple.Direction.REVERSE)
            .leftRearMotorDirection(DcMotorSimple.Direction.REVERSE)
            .rightFrontMotorDirection(DcMotorSimple.Direction.FORWARD)
            .rightRearMotorDirection(DcMotorSimple.Direction.FORWARD)
            .xVelocity(73.32)//75.10//57.49//61.78
            .yVelocity(57.25);//58.18//45.39//49.90 //(24.53);


    public static PinpointConstants localizerConstants = new PinpointConstants()
            .forwardPodY(-7)//(5.4)//(0)
            .strafePodX(-0.125)//(0)//(-5.4)
            .distanceUnit(DistanceUnit.INCH)
            .hardwareMapName("odo")
            .encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD)
            .forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection.FORWARD)
            .strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection.REVERSED);

    // defaults: (0.995, 0.1, 0.1, 0.007, 100, 1, 10, 1);
    public static PathConstraints pathConstraints = new PathConstraints(0.81, 0.3, 0.3, 0.2, 50, 1, 10, 1);//(0.95, 100, 1, 1);

    public static Follower createFollower(HardwareMap hardwareMap) {
        return new FollowerBuilder(followerConstants, hardwareMap)
                .pathConstraints(pathConstraints)
                .mecanumDrivetrain(driveConstants)
                .pinpointLocalizer(localizerConstants)
                .build();
    }
}