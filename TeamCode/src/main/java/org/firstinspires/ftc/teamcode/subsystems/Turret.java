package org.firstinspires.ftc.teamcode.subsystems;

import com.bylazar.configurables.annotations.Configurable;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.teamcode.utilities.Alliance;

import dev.nextftc.control.ControlSystem;
import dev.nextftc.control.KineticState;
import dev.nextftc.core.components.Component;
import dev.nextftc.ftc.ActiveOpMode;


@Configurable
public class Turret implements Component {
    CRServo turretLeft, turretRight;
    DcMotor thruTurret;
    private TouchSensor limitSwitch;
    Pose currentPose, limelightPose;
    Vector currentVelocity;
    public enum TurretState {
        OFF,
        AUTO_OFF,
        FORWARD,
        AUTO,
        FIXED,
        MOVE_N_SHOOT
    }
    TurretState turretState = TurretState.AUTO;
    Alliance alliance;
    private double turretAngleCached = 0;
    private double fieldCentricGoalAngle, goalX, goalY, turretPower, turretGoalNotInLimits, heading;
    private KineticState ZERO_ANGLE = new KineticState(0);
    private KineticState FIXED_ANGLE = new KineticState(-95);
    private KineticState angleAfterOffset = new KineticState(0);
    public static double turretOffset = 0;
    public static double AUTON_RED_SHOOT_ANGLE_CLOSE = -139; //-92 -95
    public static double AUTON_BLUE_SHOOT_ANGLE_CLOSE = 138;
    public static double AUTON_RED_SHOOT_ANGLE_FAR = -114; //-92 -95
    public static double AUTON_BLUE_SHOOT_ANGLE_FAR = 114;
    public boolean hasBeenReset = false;
    public boolean turretPressedAndReset = false;
    public int turretZone;
    public double turretZoneMargin = 8.5, midPointX = 72, midPointY = 72, farZoneHeight = 24, endPointY = 144;

    double botY, botX;
    public static double TURRET_PID_KP = 0.008;//0.01; //0.038; //0.017;
    public static double TURRET_PID_KD = 1.0;//0.1;//0.001;//0.2; //0.01;
    public static double TURRET_PID_KS = 0.09;//0.08;
    public static double TURRET_PID_KI = 10;//0;//0.000000000000000000001;//0.0;
    private static final double LEFT_TURRET_LIMIT = -190, RIGHT_TURRET_LIMIT = 190;
    private double shootingGoal = 0;
    public static double TURRET_ANGLE_GO_FAST = 20;//3;
    public static double TURRET_POWER_GO_FAST = 0.9;
    private static double TURRET_POWER_LIMIT = 0.9, TURRET_ANGLE_DEADZONE = 0.5, TURRET_POWER_MIN = 0.05;//D: 1
    public static double TURRET_TICKS_TO_DEGREES = (double) 1007616 /3240; // THIS WAS FOUND MATHEMATICALLY DO NOT CHANGE
    ControlSystem turretPID, turretSecPID;


    @Override
    public void preInit() {
        limitSwitch = ActiveOpMode.hardwareMap().get(TouchSensor.class, "limitSwitch");
        turretLeft = ActiveOpMode.hardwareMap().get(CRServo.class, "turretLeft");
        turretRight = ActiveOpMode.hardwareMap().get(CRServo.class, "turretRight");

        turretLeft.setDirection(DcMotorSimple.Direction.REVERSE);
        turretRight.setDirection(DcMotorSimple.Direction.REVERSE);

        thruTurret = ActiveOpMode.hardwareMap().get(DcMotor.class, "intake");
    }

    @Override
    public void postInit() {
        turretState = TurretState.AUTO;
        turretPID = ControlSystem.builder()
                .posPid(TURRET_PID_KP, TURRET_PID_KI, TURRET_PID_KD)
                //.basicFF(0, 0, TURRET_PID_KS)
                .build();
        turretGoalNotInLimits = 0;
    }

    public void zeroTurret() {
        thruTurret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        thruTurret.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    public void setTurretPower(double power) {
        turretLeft.setPower(power);
        turretRight.setPower(power);
    }

    public void update() {
        //update cached information
        turretAngleCached=((double) thruTurret.getCurrentPosition()) / TURRET_TICKS_TO_DEGREES;

        turretPID = ControlSystem.builder()
                .posPid(TURRET_PID_KP, TURRET_PID_KI, TURRET_PID_KD)
                .build();

        if (turretState == TurretState.AUTO) {
            turretPID.setGoal(getAutoAimGoalAngle());
            angleAfterOffset = new KineticState((2*getTurretGoal())- turretOffset);
        } else if (turretState == TurretState.FORWARD) {
            turretPID.setGoal(ZERO_ANGLE);
        } else if (turretState == TurretState.FIXED){  //This is the autonomous fixed position for shooting
            turretPID.setGoal(FIXED_ANGLE);
        }else if(turretState == TurretState.MOVE_N_SHOOT){
            turretPID.setGoal(new KineticState(shootingGoal));
        } else {
            // this is when the TurretState is Off
            turretPower = 0;
        }
        if (Math.abs(this.getTurretAngle() - this.getTurretGoal()) < TURRET_ANGLE_DEADZONE) {
            turretPower = 0;
        }else if (TurretState.AUTO_OFF != turretState){
            //calculate the turret power because both use it
            turretPower = turretPID.calculate(new KineticState(this.getTurretAngle()));
            //if we are too far away from being pointed away from the goal, go fast.
            if(Math.abs(this.getTurretAngle() - this.getTurretGoal()) > TURRET_ANGLE_GO_FAST){
                turretPower = Math.signum(turretPower) * TURRET_POWER_GO_FAST;
            } else {
                //if we are close use the PID to line up carefully
                turretPower = turretPower + TURRET_PID_KS * Math.signum(turretPower);
            }
        }

        botY = this.currentPose.getY();
        botX = this.currentPose.getX();
         //New code for turning off/on for launch zones
        if(TurretState.AUTO == turretState || TurretState.AUTO_OFF == turretState) {
            if (botX < midPointX && botY >= -botX + endPointY - turretZoneMargin) {
                turretState = TurretState.AUTO;
                turretZone = 1;
            } else if (botX >= midPointX && botY >= botX - turretZoneMargin) {
                turretState = TurretState.AUTO;
                turretZone = 2;
            } else if (botX < midPointX && botY <= botX - midPointX + farZoneHeight + turretZoneMargin) {
                turretState = TurretState.AUTO;
                turretZone = 3;
            } else if (botX >= midPointX && botY <= -botX + midPointX + farZoneHeight + turretZoneMargin) {
                turretState = TurretState.AUTO;
                turretZone = 4;
            } else {
                turretState = TurretState.AUTO_OFF;
                turretZone = 0;
            }
        }

        // limit the turret power to our Turret Power Limit
        if (Math.abs(turretPower) < TURRET_POWER_MIN) {
            turretPower = 0;
        }

        if (turretState != TurretState.OFF) {
            this.setTurretPower(turretPower);
        }

        ActiveOpMode.telemetry().addData("TURRET state", turretState);
        ActiveOpMode.telemetry().addData("TURRET zone", turretZone);
        ActiveOpMode.telemetry().addData("TURRET pose", this.currentPose);
        ActiveOpMode.telemetry().addData("TURRET goal", turretPID.getGoal().component1());
        ActiveOpMode.telemetry().addData("TURRET power", turretPower);
        ActiveOpMode.telemetry().addData("TURRET angle", this.getTurretAngle());
        ActiveOpMode.telemetry().addData("TURRET lim pressed", hasBeenReset);
        ActiveOpMode.telemetry().addData("TURRET lim has been pressed", turretPressedAndReset);
    }

    /**
     * @return: KineticState of goal, for auto-aim.
     */
    public KineticState getAutoAimGoalAngle() {
        if (currentPose != null) {
            fieldCentricGoalAngle = Math.atan2((goalY - this.currentPose.getY()), (goalX - this.currentPose.getX())); // IN RADS
            turretGoalNotInLimits = Math.toDegrees(normalizeAngle(fieldCentricGoalAngle - this.currentPose.getHeading() + Math.PI));
            return new KineticState(this.putInTurretLimits(turretGoalNotInLimits));


        } else {
            return ZERO_ANGLE;
        }
    }

    /**
     *
     * @param goal: goal that you are putting within turret limits
     * @return goal: which is within turret limits
     */
    public double putInTurretLimits(double goal) {
        if (goal > RIGHT_TURRET_LIMIT || goal < LEFT_TURRET_LIMIT) {
            if (goal > RIGHT_TURRET_LIMIT) {
                goal = RIGHT_TURRET_LIMIT;
            } else {
                goal = LEFT_TURRET_LIMIT;
            }
        }
        return goal;
    }

    /**
     *
     * @param pose: sets the current pose, used for auto-aim calculations
     *            NEEDS TO BE DONE EVERY LOOP
     */
    public void setCurrentPose(Pose pose, Vector velocity, double offset) {
        this.currentPose = pose;
        this.currentVelocity = velocity;
        this.turretOffset = offset;
    }

    /**
     *
     * @param goal: goal angle for turret to face
     */
    public void setTurretAngle(double goal) {
        turretState = TurretState.FORWARD;
        turretPID.setGoal(new KineticState(goal));
    }

    public void setTurretShootAngle(double goal){
        shootingGoal = normalizeAngle(goal + 180);
        turretState = TurretState.MOVE_N_SHOOT;
        turretPID.setGoal(new KineticState(shootingGoal));
    }

    public TurretState getTurretState() {
        return turretState;
    }

    /**
     *
     * @return turret angle after converting from ticks
     * IN DEGREES
     */
    public double getTurretAngle() {
        return turretAngleCached; //-((double) thruTurret.getCurrentPosition()) / TURRET_TICKS_TO_DEGREES;
    }

    /**
     *
     * @return current turret goal NOT in physical limits. in degrees
     */
    public double getTurretGoalNotInLimits() {
        return turretGoalNotInLimits;
    }

    /**
     *
     * @return current turret goal IN physical limits. in degrees
     */
    public double getTurretGoal() {
        return turretPID.getGoal().getPosition();
    }

    public static double normalizeAngle(double angleRad) {
        // Ensure the angle is within the (-π, π] range
        angleRad %= (2 * Math.PI); // Take the modulo with 2π
        if (angleRad > Math.PI) {
            angleRad -= (2 * Math.PI); // Subtract 2π if greater than π
        } else if (angleRad <= -Math.PI) {
            angleRad += (2 * Math.PI); // Add 2π if less than or equal to -π
        }
        return angleRad;
    }

    /**
     * Sets the physical goal position based on alliance. Must be called on init
     * @param all: alliance we're on
     */
    public void setAlliance(Alliance all) {
        this.alliance = all;
        if (this.alliance == Alliance.RED) {
            goalX = 144;//142
            goalY = 144;
        } else {
            goalX = 0;//0
            goalY = 144;
        }
    }

    /**
     * Adjust goal after initialization, meant for use with shooting on the move
     * @param x amount to adjust goal in the x direction
     * @param y amount to adjust goal in the y direction
     */
    private void adjustGoalPosition(double x, double y) {
        if (this.alliance == Alliance.RED) {
            goalX = 144 + x;//142
            goalY = 144 + y;
        } else {
            goalX = 0 + x;//0
            goalY = 144 + y;
        }
    }

    /**
     * Use this method to enable shooting on the move. Must be called each loop
     * @param currentVelocity MUST be a Pedro Vector (in/s).
     * use follower.getVelocity()
     */
    public void shootOnTheMove(Vector currentVelocity) {
        // these are negative because we want the goal position to adjust in the opposite way of the
        adjustGoalPosition(-currentVelocity.getXComponent(), -currentVelocity.getYComponent());
    }

    public void setFixedAngleCustom(double angle) {
        turretState = TurretState.FIXED;
        FIXED_ANGLE = new KineticState(angle);
    }

    public void setFixedAngle(Alliance alliance, boolean isClose) {
        turretState = TurretState.FIXED;
        if (isClose) {
            if (alliance == Alliance.BLUE) {
                FIXED_ANGLE = new KineticState(AUTON_BLUE_SHOOT_ANGLE_CLOSE);
            } else {
                FIXED_ANGLE = new KineticState(AUTON_RED_SHOOT_ANGLE_CLOSE);
            }
        } else {
            if (alliance == Alliance.BLUE) {
                FIXED_ANGLE = new KineticState(AUTON_BLUE_SHOOT_ANGLE_FAR);
            } else {
                FIXED_ANGLE = new KineticState(AUTON_RED_SHOOT_ANGLE_FAR);
            }
        }
    }
    public void setTurretStateOff() {
        turretState = TurretState.OFF;
    }
    public void setTurretStateAuto() {
        turretState = TurretState.AUTO;
    }

    public void setTurretStateFixed() {
        turretState = TurretState.FIXED;
    }

    public void turretStateForward() {
        switch (turretState) {
            case AUTO:
                this.turretState = TurretState.FORWARD;
                break;
            case FORWARD:
                this.turretState = TurretState.OFF; break;
            case OFF:
                this.turretState = TurretState.AUTO; break;
        }
    }

    public void turretStateBackward() {
        switch (turretState) {
            case AUTO:
                this.turretState = TurretState.OFF;
                break;
            case FORWARD:
                this.turretState = TurretState.AUTO;
                break;
            case OFF:
                this.turretState = TurretState.FORWARD;
                break;
        }
    }

}
