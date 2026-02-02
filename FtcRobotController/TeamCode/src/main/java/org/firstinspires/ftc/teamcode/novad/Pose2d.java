package org.firstinspires.ftc.teamcode.novad;

/**
 * 2D Pose representation (x, y, heading)
 * Immutable data class for robot position.
 */
public class Pose2d {
    public final double x;       // inches
    public final double y;       // inches
    public final double heading; // radians

    public Pose2d(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    public Pose2d() {
        this(0, 0, 0);
    }

    /**
     * Calculate error from this pose to target pose
     */
    public Pose2d minus(Pose2d other) {
        return new Pose2d(
            this.x - other.x,
            this.y - other.y,
            normalizeAngle(this.heading - other.heading)
        );
    }

    /**
     * Distance to another pose (ignoring heading)
     */
    public double distanceTo(Pose2d other) {
        double dx = this.x - other.x;
        double dy = this.y - other.y;
        return Math.sqrt(dx * dx + dy * dy);
    }

    /**
     * Normalize angle to [-π, π]
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    @Override
    public String toString() {
        return String.format("Pose2d(x=%.2f, y=%.2f, h=%.1f°)", 
            x, y, Math.toDegrees(heading));
    }
}
