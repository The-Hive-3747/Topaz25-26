package org.firstinspires.ftc.teamcode.utilities;

public class BotStats { //used in datalogger to transfer information from aimbot and turret code

    public double aimVelocity;
    public double aimHoodPos;
    public double turretAngle;
    public BotStats( double aimVelocity, double aimHoodPos, double turretAngle) {
        this.aimVelocity = aimVelocity;
        this.aimHoodPos = aimHoodPos;
        this.turretAngle = turretAngle;
    }
}
