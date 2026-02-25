package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Gamepad;

import dev.nextftc.ftc.ActiveOpMode;
import dev.nextftc.core.components.Component;

public class FieldCentricDrive implements Component {
    double offset = 0;
    DcMotor frontLeft, frontRight, backLeft, backRight;
    Gamepad gamepad;


    @Override
    public void postInit() {
        frontLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, "frontLeftMotor");
        frontRight = ActiveOpMode.hardwareMap().get(DcMotor.class, "frontRightMotor");
        backLeft = ActiveOpMode.hardwareMap().get(DcMotor.class, "backLeftMotor");
        backRight = ActiveOpMode.hardwareMap().get(DcMotor.class, "backRightMotor");
        gamepad = ActiveOpMode.gamepad1();
        frontLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        backLeft.setDirection(DcMotorSimple.Direction.REVERSE);
    }


    /**
     *
     * @param heading: current heading of robot (without offset)
     * @param slowMode: slow mode multiplier (double)
     */
    public void update(double heading, double slowMode) {
                    //limelightComponent.update();
//                    if(limelightComponent.hasTarget()) {
//                        double limelightX = limelightComponent.getTargetX();
//                        double limelightY = limelightComponent.getTargetY();
//                        double limelightHeading = limelightComponent.getTargetHeading();
//                    }


                    double y = -gamepad.left_stick_y * slowMode; //y
                    double x = gamepad.left_stick_x * slowMode;
                    double rx = gamepad.right_stick_x * slowMode; //rx

                    double botHeading = this.getHeading(heading);
                    double rotX = x * Math.cos(-botHeading) - y * Math.sin(-botHeading);
                    double rotY = x * Math.sin(-botHeading) + y * Math.cos(-botHeading);

                    rotX = rotX * 1.1;

                    double denominator = Math.max(Math.abs(rotY) + Math.abs(rotX) + Math.abs(rx), 1);
                    double frontLeftPower = (rotY + rotX + rx) / denominator;//rotY + rotX + rx
                    double backLeftPower = (rotY - rotX + rx) / denominator;//rotY - rotX + rx
                    double frontRightPower = (rotY - rotX - rx) / denominator;//rotY - rotX - rx
                    double backRightPower = (rotY + rotX - rx) / denominator;//rotY + rotX - rx

                    frontLeft.setPower(frontLeftPower);
                    frontRight.setPower(frontRightPower);
                    backRight.setPower(backRightPower);
                    backLeft.setPower(backLeftPower);

    }

    /**
     *
     * @param gamepadNum: allows you to change the gamepad. call AFTER init.
     */
    public void setGamepad(int gamepadNum) {
        if (gamepadNum == 2) {
            gamepad = ActiveOpMode.gamepad2();
        } else {
            gamepad = ActiveOpMode.gamepad1();
        }
    }

    /**
     *
     * @param offset: use this with the robot's current heading to reset the field oriented drive
     */
    public void setOffset(double offset) {
        this.offset = offset;
    }

    /**
     *
     * @param heading: heading of robot
     * @return: heading with offset
     */
    private double getHeading(double heading) {
        return heading - this.offset;
    }
}