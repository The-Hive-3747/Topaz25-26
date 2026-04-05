package org.firstinspires.ftc.teamcode.utilities;

public class ShooterKinematics {
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
     * with a fixed vertical launch angle.
     *
     * @param robotX          Robot X position (inches)
     * @param robotY          Robot Y position (inches)
     * @param robotVx         Robot's current X velocity (inches/sec)
     * @param robotVy         Robot's current Y velocity (inches/sec)
     * @param goalX           Goal X position (inches)
     * @param goalY           Goal Y position (inches)
     * @param targetDeltaZ    Height of the goal relative to the shooter (inches)
     * @param launchAngleDeg  Fixed pitch angle of the shooter (degrees)
     * @param flyWheelCircum  Circumference of the flywheel (inches)
     * @return                ShotParameters containing target heading, RPM, and calculated flight time
     */
    public static ShotParameters calculate3DMovingShot(
            double robotX, double robotY, double robotVx, double robotVy,
            double goalX, double goalY, double targetDeltaZ,
            double launchAngleDeg, double flyWheelCircum) {

        final double GRAVITY = 386.09; // Gravity in inches/sec^2
        double theta = Math.toRadians(launchAngleDeg);
        double tanTheta = Math.tan(theta);

        double deltaX = goalX - robotX;
        double deltaY = goalY - robotY;

        // Binary search for the correct Time of Flight (t)
        double tMin = 0.01;
        double tMax = 4.0; // 4 seconds is well beyond a standard FTC shot
        double t = 2.0;

        // 40 iterations provides extremely high precision (microsecond accuracy)
        for (int i = 0; i < 40; i++) {
            t = (tMin + tMax) / 2.0;

            // 1. Calculate required absolute horizontal velocity to reach goal in time 't'
            double absVx = deltaX / t;
            double absVy = deltaY / t;

            // 2. Subtract robot velocity to find required relative horizontal velocity
            double relVx = absVx - robotVx;
            double relVy = absVy - robotVy;

            // 3. Magnitude of required relative horizontal velocity
            double relVxy = Math.hypot(relVx, relVy);

            // 4. Compare Provided Vertical Velocity vs Required Vertical Velocity
            double providedVz = relVxy * tanTheta;
            double requiredVz = (targetDeltaZ / t) + (0.5 * GRAVITY * t);

            if (providedVz > requiredVz) {
                // Too much vertical velocity means our horizontal velocity is too high.
                // We need a longer time of flight to reduce horizontal speed requirements.
                tMin = t;
            } else {
                // Not enough vertical velocity, we need to get there faster.
                tMax = t;
            }
        }

        // --- Post-Search Final Calculations ---

        // Recalculate final horizontal velocities with our solved 't'
        double finalAbsVx = deltaX / t;
        double finalAbsVy = deltaY / t;
        double finalRelVx = finalAbsVx - robotVx;
        double finalRelVy = finalAbsVy - robotVy;

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
