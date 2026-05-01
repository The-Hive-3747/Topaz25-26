package org.firstinspires.ftc.teamcode.subsystems;

import static org.firstinspires.ftc.teamcode.opmodes.TopazTeleopMoveNShoot.FAR_ZONE_THRESHOLD_IN;

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
    Pose currentPose = new Pose (0,0,0), limelightPose;
    Vector currentVelocity;

    public void setMoveNShoot() {
        isMoveNShoot = true;
        turretState = TurretState.MOVE_N_SHOOT;
    }

    public enum TurretState {
        OFF,
        AUTO_OFF,
        FORWARD,
        AUTO,
        FIXED,
        MOVE_N_SHOOT,
        REZEROING_LEFT,
        REZEROING_RIGHT
    }
    TurretState turretState = TurretState.MOVE_N_SHOOT;
    Alliance alliance;
    private double turretAngleCached = 0;
    private double heightToFlywheelTop = 11.25; //in inches
    private double heightToGoal = 40; //in inches
    private double flywheelCircumMm = 71; //this is of the inner edge in millimeters
    private double flywheelCircumInch = 2.79528;
    private double fieldCentricGoalAngle, goalX, goalY, turretPower, turretGoalNotInLimits, heading;
    private final KineticState ZERO_ANGLE = new KineticState(0);
    private KineticState FIXED_ANGLE = new KineticState(-95);
    public static double turretOffset = 0;
    public static double AUTON_RED_SHOOT_ANGLE_CLOSE = -125;//-144; //-92 -95
    public static double AUTON_BLUE_SHOOT_ANGLE_CLOSE = 144;
    public static double AUTON_RED_SHOOT_ANGLE_FAR = -115; //-92 -95
    public static double AUTON_BLUE_SHOOT_ANGLE_FAR = 114; //113.2; //114 112
    public static double LEFT_LIMIT_TICKS = 685;
    public static double RIGHT_LIMIT_TICKS = -856;
    //public static double turretRezeroTolerance = 0.5; //TODO: rezero tolerance has to match deadzone
    public double REZERO_POWER = 0.25;//0.16;//0.11;//0.2;//0.1;//0.05;
    public boolean hasBeenReset = false;
    public boolean turretPressedAndReset = false;
    public boolean turretRezeroed = false;
    public boolean turretFindingSwitch = false;
    public static boolean isTeleop = true;
    public boolean isMoveNShoot = true;
    public int turretZone;
    public double turretZoneMarginBack = 17, turretZoneMargin = 11, midPointX = 72, midPointY = 72, farZoneHeight = 24, endPointY = 144;// turretzonemargin:11//8.5
    double botY, botX;
    public static double TURRET_PID_KS_CURRENT = 0.0;
    public static double TURRET_PID_KP_SOTM = 0.0011;//0.0015;//0.0075;//0.01; //0.038; //0.017;
    public static double TURRET_PID_KD_SOTM = 0.08;//1.0;//1.0;//0.1;//0.001;//0.2; //0.01;
    public static double TURRET_PID_KS_SOTM = 0.095;//0.125;//0.11; //0.10; //0.09;//0.08;
    public static double TURRET_PID_KI_SOTM = 0.0;//0.0000000000001;//0.00000000000000001;//10;//0;//0.000000000000000000001;//0.0;

    public static double TURRET_PID_KP_RED_CLOSE = 0.0011;
    public static double TURRET_PID_KD_RED_CLOSE = 0.08;
    public static double TURRET_PID_KS_RED_CLOSE = 0.095;
    public static double TURRET_PID_KI_RED_CLOSE = 0.0;
    public static double TURRET_PID_KP_RED_FAR = 0.0015; //not found
    public static double TURRET_PID_KD_RED_FAR = 1;
    public static double TURRET_PID_KS_RED_FAR_POSITIVE = 0.085;
    public static double TURRET_PID_KS_RED_FAR_NEGATIVE = 0.115;
    public static double TURRET_PID_KI_RED_FAR = 0.0;

    public static double TURRET_PID_KP_BLUE_CLOSE = 0.0015; //not found
    public static double TURRET_PID_KD_BLUE_CLOSE = 1.0;
    public static double TURRET_PID_KS_BLUE_CLOSE = 0.125;
    public static double TURRET_PID_KI_BLUE_CLOSE = 0.0000000000001;
    public static double TURRET_PID_KP_BLUE_FAR = 0.0015; //mostly tested
    public static double TURRET_PID_KD_BLUE_FAR = 1;
    public static double TURRET_PID_KS_BLUE_FAR_POSITIVE = 0.135;
    public static double TURRET_PID_KS_BLUE_FAR_NEGATIVE = 0.065;
    public static double TURRET_PID_KI_BLUE_FAR = 0.0000000000001;

    private static final double LEFT_TURRET_LIMIT = -190, RIGHT_TURRET_LIMIT = 190;
    private double shootingGoal = 0;
    public static double TURRET_ANGLE_GO_FAST = 25;//3;
    public static double TURRET_POWER_GO_FAST = 0.8;
    private static double TURRET_POWER_LIMIT = 0.9, TURRET_ANGLE_DEADZONE = 1, TURRET_POWER_MIN = 0.05;//D: 0.5//1
    public static double TURRET_TICKS_TO_DEGREES = (double) 1007616 /3240; // THIS WAS FOUND MATHEMATICALLY DO NOT CHANGE
    public double turretRobotHeadingInDeg, robotFieldHeadingInRads, shootOnTheMoveHeadingInRads;
    ControlSystem turretPIDSOTM, turretPIDRedClose, turretPIDRedFar, turretPIDBlueClose, turretPIDBlueFar;
    //private KineticState turretRobotHeading;
    PIDState pidState = PIDState.BLUE_CLOSE_PID;
    public enum PIDState {
        RED_FAR_PID,
        RED_CLOSE_PID,
        BLUE_FAR_PID,
        BLUE_CLOSE_PID,
        TELEOP_PID
    }

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
        //turretState = TurretState.AUTO;
        turretPIDSOTM = ControlSystem.builder()
                .posPid(TURRET_PID_KP_SOTM, TURRET_PID_KI_SOTM, TURRET_PID_KD_SOTM)
                //.basicFF(0, 0, TURRET_PID_KS)
                .build();

        turretPIDRedFar = ControlSystem.builder()
                .posPid(TURRET_PID_KP_RED_FAR, TURRET_PID_KI_RED_FAR, TURRET_PID_KD_RED_FAR)
                //.basicFF(0, 0, TURRET_PID_KS)
                .build();

        turretPIDRedClose = ControlSystem.builder()
                .posPid(TURRET_PID_KP_RED_CLOSE, TURRET_PID_KI_RED_CLOSE, TURRET_PID_KD_RED_CLOSE)
                //.basicFF(0, 0, TURRET_PID_KS)
                .build();

        turretPIDBlueFar = ControlSystem.builder()
                .posPid(TURRET_PID_KP_BLUE_FAR, TURRET_PID_KI_BLUE_FAR, TURRET_PID_KD_BLUE_FAR)
                //.basicFF(0, 0, TURRET_PID_KS)
                .build();

        turretPIDBlueClose = ControlSystem.builder()
                .posPid(TURRET_PID_KP_BLUE_CLOSE, TURRET_PID_KI_BLUE_CLOSE, TURRET_PID_KD_BLUE_CLOSE)
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
        robotFieldHeadingInRads = currentPose.getHeading();
        /*turretPID = ControlSystem.builder()
                .posPid(TURRET_PID_KP, TURRET_PID_KI, TURRET_PID_KD)
                .build();*/

        if (turretState == TurretState.AUTO) {
            turretPIDSOTM.setGoal(getAutoAimGoalAngle());
        } else if (turretState == TurretState.FORWARD) {
            turretPIDSOTM.setGoal(ZERO_ANGLE);
            turretPIDRedClose.setGoal(ZERO_ANGLE);
            turretPIDRedFar.setGoal(ZERO_ANGLE);
            turretPIDBlueClose.setGoal(ZERO_ANGLE);
            turretPIDBlueFar.setGoal(ZERO_ANGLE);

        } else if (turretState == TurretState.FIXED){  //This is the autonomous fixed position for shooting
            turretPIDSOTM.setGoal(FIXED_ANGLE);
            turretPIDRedClose.setGoal(FIXED_ANGLE);
            turretPIDRedFar.setGoal(FIXED_ANGLE);
            turretPIDBlueClose.setGoal(FIXED_ANGLE);
            turretPIDBlueFar.setGoal(FIXED_ANGLE);
        }else if(turretState == TurretState.MOVE_N_SHOOT){
            //turretPID.setGoal(new KineticState(shootingGoal));
            turretPIDSOTM.setGoal(convertFieldHeadingRadsToKineticStateUsingDeg(shootOnTheMoveHeadingInRads));
            turretPIDRedFar.setGoal(convertFieldHeadingRadsToKineticStateUsingDeg(shootOnTheMoveHeadingInRads));
            turretPIDRedClose.setGoal(convertFieldHeadingRadsToKineticStateUsingDeg(shootOnTheMoveHeadingInRads));
            turretPIDBlueFar.setGoal(convertFieldHeadingRadsToKineticStateUsingDeg(shootOnTheMoveHeadingInRads));
            turretPIDBlueClose.setGoal(convertFieldHeadingRadsToKineticStateUsingDeg(shootOnTheMoveHeadingInRads));

        } else if(turretState == TurretState.REZEROING_RIGHT){
            if (!turretRezeroed && !limitSwitch.isPressed()) { //cannot fix as there is no way to reverse polarity on limit switch.
                turretFindingSwitch = false;
                //zeroTurret();
                turretPIDSOTM.setGoal(new KineticState(getTurretAngle() -LEFT_LIMIT_TICKS/TURRET_TICKS_TO_DEGREES));
                turretRezeroed = true;
            }
            if (turretRezeroed && Math.abs(this.getTurretAngle() - this.getTurretGoal()) <= TURRET_ANGLE_DEADZONE) {
                this.zeroTurret();
                if (isMoveNShoot) {
                    turretState = TurretState.MOVE_N_SHOOT;
                }else {
                    turretState = TurretState.AUTO;
                }
                turretRezeroed = false;
            }
        }else if(turretState == TurretState.REZEROING_LEFT){ //cannot fix as there is no way to reverse polarity on limit switch.
            if (!turretRezeroed && !limitSwitch.isPressed()) {
                turretFindingSwitch = false;
                //zeroTurret();
                turretPIDSOTM.setGoal(new KineticState(getTurretAngle() -RIGHT_LIMIT_TICKS/TURRET_TICKS_TO_DEGREES));
                turretRezeroed = true;
            }
            if (turretRezeroed && Math.abs(this.getTurretAngle() - this.getTurretGoal()) <= TURRET_ANGLE_DEADZONE) {
                this.zeroTurret();
                if (isMoveNShoot) {
                    turretState = TurretState.MOVE_N_SHOOT;
                }else {
                    turretState = TurretState.AUTO;
                }
                turretRezeroed = false;
            }
        } else {
            // this is when the TurretState is Off
            turretPower = 0;
        }
        if (Math.abs(this.getTurretAngle() - this.getTurretGoal()) < TURRET_ANGLE_DEADZONE &&
                turretState != TurretState.REZEROING_RIGHT && turretState != TurretState.REZEROING_LEFT) {
            turretPower = 0;
        } else if (TurretState.AUTO_OFF != turretState  && !turretFindingSwitch) {
            //calculate the turret power because both use it
            if (isTeleop) {
                if (thruTurret.getCurrentPosition()<0) { //turret is on negative side
                    if (currentPose.getY()<FAR_ZONE_THRESHOLD_IN) {
                        pidState = PIDState.RED_FAR_PID;
                        turretPower = turretPIDRedFar.calculate(new KineticState(this.getTurretAngle()));
                        if ((this.getTurretGoal() - this.getTurretAngle()) < 0) {
                            TURRET_PID_KS_CURRENT = TURRET_PID_KS_RED_FAR_POSITIVE;
                        } else {
                            TURRET_PID_KS_CURRENT = TURRET_PID_KS_RED_FAR_NEGATIVE;
                        }
                    } else {
                        pidState = PIDState.RED_CLOSE_PID;
                        turretPower = turretPIDRedClose.calculate(new KineticState(this.getTurretAngle()));
                        TURRET_PID_KS_CURRENT = TURRET_PID_KS_RED_CLOSE;
                    }
                } else if (thruTurret.getCurrentPosition()>=0) { //turret is in positive zone
                    if (currentPose.getY()<FAR_ZONE_THRESHOLD_IN) {
                        pidState = PIDState.BLUE_FAR_PID;
                        turretPower = turretPIDBlueFar.calculate(new KineticState(this.getTurretAngle()));
                        if ((this.getTurretGoal() - this.getTurretAngle()) > 0) {
                            TURRET_PID_KS_CURRENT = TURRET_PID_KS_BLUE_FAR_POSITIVE;
                        } else {
                            TURRET_PID_KS_CURRENT = TURRET_PID_KS_BLUE_FAR_NEGATIVE;
                        }
                    } else { //default
                        pidState = PIDState.BLUE_CLOSE_PID;
                        turretPower = turretPIDBlueClose.calculate(new KineticState(this.getTurretAngle()));
                        TURRET_PID_KS_CURRENT = TURRET_PID_KS_BLUE_CLOSE;
                    }
                } else {
                pidState = PIDState.TELEOP_PID;
                turretPower = turretPIDSOTM.calculate(new KineticState(this.getTurretAngle()));
                TURRET_PID_KS_CURRENT = TURRET_PID_KS_SOTM;
                }
            } else if (Alliance.RED == alliance) {
                if (currentPose.getY()<FAR_ZONE_THRESHOLD_IN) {
                    pidState = PIDState.RED_FAR_PID;
                    turretPower = turretPIDRedFar.calculate(new KineticState(this.getTurretAngle()));
                    if ((this.getTurretGoal() - this.getTurretAngle()) < 0) {
                        TURRET_PID_KS_CURRENT = TURRET_PID_KS_RED_FAR_POSITIVE;
                    } else {
                        TURRET_PID_KS_CURRENT = TURRET_PID_KS_RED_FAR_NEGATIVE;
                    }
                } else {
                    pidState = PIDState.RED_CLOSE_PID;
                    turretPower = turretPIDRedClose.calculate(new KineticState(this.getTurretAngle()));
                    TURRET_PID_KS_CURRENT = TURRET_PID_KS_RED_CLOSE;
                }
            } else {
                if (currentPose.getY()<FAR_ZONE_THRESHOLD_IN) {
                    pidState = PIDState.BLUE_FAR_PID;
                    turretPower = turretPIDBlueFar.calculate(new KineticState(this.getTurretAngle()));
                    if ((this.getTurretGoal() - this.getTurretAngle()) > 0) {
                        TURRET_PID_KS_CURRENT = TURRET_PID_KS_BLUE_FAR_POSITIVE;
                    } else {
                        TURRET_PID_KS_CURRENT = TURRET_PID_KS_BLUE_FAR_NEGATIVE;
                    }

                } else {
                    pidState = PIDState.BLUE_CLOSE_PID;
                    turretPower = turretPIDBlueClose.calculate(new KineticState(this.getTurretAngle()));
                    TURRET_PID_KS_CURRENT = TURRET_PID_KS_BLUE_CLOSE;
                }
            }


            //if we are too far away from being pointed away from the goal, go fast.
            if(Math.abs(this.getTurretAngle() - this.getTurretGoal()) > TURRET_ANGLE_GO_FAST){
                turretPower = Math.signum(turretPower) * TURRET_POWER_GO_FAST;
            } else {
                //if we are close use the PID to line up carefully
                turretPower = turretPower + TURRET_PID_KS_CURRENT * Math.signum(turretPower);
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
            } else if (botX < midPointX && botY <= botX - midPointX + farZoneHeight + turretZoneMarginBack) {
                turretState = TurretState.AUTO;
                turretZone = 3;
            } else if (botX >= midPointX && botY <= -botX + midPointX + farZoneHeight + turretZoneMarginBack) {
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
        if (turretState != TurretState.OFF && !turretFindingSwitch) {
            this.setTurretPower(turretPower);
        }
    }

    public void telemetry() {
        ActiveOpMode.telemetry().addLine("---- Turret ----");
        ActiveOpMode.telemetry().addData("Turret state", turretState);
        ActiveOpMode.telemetry().addData("Turret PID state", pidState);
        ActiveOpMode.telemetry().addData("Turret power", turretPower);
        ActiveOpMode.telemetry().addData("Turret position", this.getTurretAngle());
        ActiveOpMode.telemetry().addData("Turret goal", turretPIDSOTM.getGoal().component1());
        ActiveOpMode.telemetry().addData("Turret limit switch has been pressed", turretPressedAndReset);


    }

    /**
     * @return: KineticState of goal, for auto-aim.
     */
    public KineticState getAutoAimGoalAngle() {
        if (currentPose != null) {
            fieldCentricGoalAngle = Math.atan2((goalY - this.currentPose.getY()), (goalX - this.currentPose.getX())); // IN RADS
            turretGoalNotInLimits = Math.toDegrees(normalizeAngleInRads(fieldCentricGoalAngle - this.currentPose.getHeading() + Math.PI));
            return new KineticState(this.putInTurretLimits(turretGoalNotInLimits));

        } else {
            return ZERO_ANGLE;
        }
    }
    public KineticState convertFieldHeadingRadsToKineticStateUsingDeg(double goalFieldHeadingInRads) {
        turretRobotHeadingInDeg = Math.toDegrees(normalizeAngleInRads(Math.PI - (robotFieldHeadingInRads - goalFieldHeadingInRads)));
        return new KineticState (this.putInTurretLimits(turretRobotHeadingInDeg));
    }
    public void setTurretOnTheMoveInRads(double goalFieldHeadingInRads) {
        shootOnTheMoveHeadingInRads = goalFieldHeadingInRads;
        //setTurretStateMoveNShoot();
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
    public void setCurrentPose(Pose pose) {
        this.currentPose = pose;
    }

    /**
     *
     * @param goal: goal angle for turret to face
     */
    public void setTurretAngle(double goal) {
        turretState = TurretState.FORWARD;
        turretPIDSOTM.setGoal(new KineticState(goal));
    }

    public void setTurretShootAngle(double goal){
        shootingGoal = normalizeAngleInRads(goal + 180);
        //turretState = TurretState.MOVE_N_SHOOT;
        turretPIDSOTM.setGoal(new KineticState(shootingGoal));
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
        return turretPIDSOTM.getGoal().getPosition();
    }

    public static double normalizeAngleInRads(double angleRad) {
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
            goalX = 142;//142
            goalY = 144;
        } else {
            goalX = 2;//0
            goalY = 144;
        }
    }

    public double getGoalX() {
        return goalX;
    }

    public double getGoalY() {
        return goalY;
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
        isMoveNShoot = false;
    }
    public void setTurretStateAuto() {
        turretState = TurretState.AUTO;
        isMoveNShoot = false;
    }
    public void setTurretStateMoveNShoot() {
        turretState = TurretState.MOVE_N_SHOOT;
        isMoveNShoot = true;
    }
    public void setTurretStateRezeroLeft() {
        turretRezeroed = false;
        turretState = TurretState.REZEROING_LEFT;
        setTurretPower(REZERO_POWER);
        turretFindingSwitch = true;
    }

    public void setTurretStateRezeroRight() {
        turretRezeroed = false;
        turretState = TurretState.REZEROING_RIGHT;
        setTurretPower(-REZERO_POWER);
        turretFindingSwitch = true;
    }

    public void setTeleopPID() {
        isTeleop = true;
    }
    public void setAutoPID() {
        isTeleop = false;
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
