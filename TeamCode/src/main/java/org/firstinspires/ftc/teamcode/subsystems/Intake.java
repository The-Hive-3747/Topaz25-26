package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import dev.nextftc.core.commands.Command;
import dev.nextftc.core.commands.utility.InstantCommand;
import dev.nextftc.core.commands.utility.LambdaCommand;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;

public class Intake implements Component {
    DcMotor intakeMotor;
    double INTAKE_POWER = 0.9;
    double INTAKE_SHOOTING_POWER = 0.9;
    double INTAKE_FAST = 1.0;
    double FIRE_POWER = 0.9;
    CRServo leftFireServo, rightFireServo, sideWheelServo, hood;

    @Override
    public void postInit() {
        intakeMotor = ActiveOpMode.hardwareMap().get(DcMotor.class, "transfer");
        leftFireServo = ActiveOpMode.hardwareMap().get(CRServo.class, "left_firewheel");
        sideWheelServo = ActiveOpMode.hardwareMap().get(CRServo.class, "side-wheel");
        hood = ActiveOpMode.hardwareMap().get(CRServo.class, "hoodServo");
        sideWheelServo.setDirection(CRServo.Direction.REVERSE);
        leftFireServo.setDirection(CRServo.Direction.REVERSE);
    }

    public Command startIntake = new InstantCommand(
            () -> intakeMotor.setPower(INTAKE_POWER)
    );
    public Command slowIntake = new InstantCommand(
            () -> intakeMotor.setPower(INTAKE_SHOOTING_POWER)
    );
    public Command fastIntake = new InstantCommand(
            () -> intakeMotor.setPower(INTAKE_FAST)
    );
    public Command startTransfer = new InstantCommand(
            () -> {
                leftFireServo.setPower(FIRE_POWER);
                sideWheelServo.setPower(FIRE_POWER);
            }
    );

    public Command stopIntake = new InstantCommand(
            () -> intakeMotor.setPower(0)
    );
    public Command stopTransfer = new InstantCommand(
            () -> {
                leftFireServo.setPower(0);
                sideWheelServo.setPower(0);
            }
    );


    public void update() {
        ActiveOpMode.telemetry().addData("left firewheel power", leftFireServo.getPower());
        ActiveOpMode.telemetry().addData("sidewheel power", sideWheelServo.getPower());

    }
}
