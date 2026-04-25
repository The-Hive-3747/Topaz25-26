package org.firstinspires.ftc.teamcode.utilities;

public class ShooterKinematicsAccel {
    public static class ShotParameters {
        public double headingRadians;
        public double flywheelRPM;
        public double timeOfFlight;

        public ShotParameters(double heading, double rpm, double tof) {
            this.headingRadians = heading;
            this.flywheelRPM = rpm;
            this.timeOfFlight = tof;
        }
    }

    /**
     * Calculates the required shooter heading and flywheel RPM to shoot on the move
     * with a fixed vertical launch angle, accounting for robot acceleration and system latency.
     *
     * @param robotX          Robot X position (inches)
     * @param robotY          Robot Y position (inches)
     * @param robotVx         Robot's current X velocity (inches/sec)
     * @param robotVy         Robot's current Y velocity (inches/sec)
     * @param robotAx         Robot's current X acceleration (inches/sec^2)
     * @param robotAy         Robot's current Y acceleration (inches/sec^2)
     * @param systemLatency   Expected delay between calculation and actual launch (seconds)
     * @param goalX           Goal X position (inches)
     * @param goalY           Goal Y position (inches)
     * @param targetDeltaZ    Height of the goal relative to the shooter (inches)
     * @param launchAngleDeg  Fixed pitch angle of the shooter (degrees)
     * @param flyWheelCircum  Circumference of the flywheel (inches)
     * @return                ShotParameters containing target heading, RPM, and calculated flight time
     */
    public static ShotParameters calculate3DMovingShot(
            double robotX, double robotY, double robotVx, double robotVy,
            double robotAx, double robotAy, double systemLatency,
            double goalX, double goalY, double targetDeltaZ,
            double launchAngleDeg, double flyWheelCircum) {

        final double GRAVITY = 386.09; // Gravity in inches/sec^2
        double theta = Math.toRadians(launchAngleDeg);
        double tanTheta = Math.tan(theta);

        // --- PREDICT LAUNCH STATE ---
        // Project the robot's position and velocity forward in time to account for
        // the delay between this calculation and the physical release of the piece.
        double launchX = robotX + (robotVx * systemLatency) + (0.5 * robotAx * Math.pow(systemLatency, 2));
        double launchY = robotY + (robotVy * systemLatency) + (0.5 * robotAy * Math.pow(systemLatency, 2));
        double launchVx = robotVx + (robotAx * systemLatency);
        double launchVy = robotVy + (robotAy * systemLatency);

        // Calculate distance from the PREDICTED launch position
        double deltaX = goalX - launchX;
        double deltaY = goalY - launchY;

        // Binary search for the correct Time of Flight (t)
        double tMin = 0.01;
        double tMax = 4.0;
        double t = 2.0;

        // 40 iterations provides microsecond accuracy
        for (int i = 0; i < 40; i++) {
            t = (tMin + tMax) / 2.0;

            // 1. Calculate required absolute horizontal velocity to reach goal in time 't'
            double absVx = deltaX / t;
            double absVy = deltaY / t;

            // 2. Subtract predicted robot launch velocity to find required relative horizontal velocity
            double relVx = absVx - launchVx;
            double relVy = absVy - launchVy;

            // 3. Magnitude of required relative horizontal velocity
            double relVxy = Math.hypot(relVx, relVy);

            // 4. Compare Provided Vertical Velocity vs Required Vertical Velocity
            double providedVz = relVxy * tanTheta;
            double requiredVz = (targetDeltaZ / t) + (0.5 * GRAVITY * t);

            if (providedVz > requiredVz) {
                tMin = t;
            } else {
                tMax = t;
            }
        }

        // --- Post-Search Final Calculations ---

        // Recalculate final horizontal velocities with our solved 't'
        double finalAbsVx = deltaX / t;
        double finalAbsVy = deltaY / t;
        double finalRelVx = finalAbsVx - launchVx;
        double finalRelVy = finalAbsVy - launchVy;

        // Required Heading (Aiming direction relative to the field)
        double targetHeading = Math.atan2(finalRelVy, finalRelVx);

        // Total launch speed in 3D space
        double relVxy = Math.hypot(finalRelVx, finalRelVy);
        double totalRelativeSpeed = relVxy / Math.cos(theta);

        // Convert linear speed to RPM
        double flywheelRPM = (totalRelativeSpeed / flyWheelCircum) * 60.0;

        return new ShotParameters(targetHeading, flywheelRPM, t);
    }
}