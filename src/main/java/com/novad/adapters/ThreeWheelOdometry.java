package com.novad.adapters;

import com.novad.interfaces.NovadOdometry;
import com.novad.util.Vector2D;

/**
 * Generic Three-Wheel Odometry Implementation
 * 
 * Use this class if you want Novad to manage your odometry directly
 * using three dead wheel encoders.
 * 
 * Wheel configuration:
 * - Left wheel: parallel to left side of robot
 * - Right wheel: parallel to right side of robot  
 * - Center wheel: perpendicular (for strafe)
 * 
 * Example:
 * <pre>
 * ThreeWheelOdometry odometry = new ThreeWheelOdometry(
 *     () -> leftEncoder.getCurrentPosition(),
 *     () -> rightEncoder.getCurrentPosition(),
 *     () -> centerEncoder.getCurrentPosition(),
 *     2.0,    // wheel diameter in inches
 *     8192,   // encoder ticks per revolution
 *     12.0,   // track width (distance between left and right wheels)
 *     6.0     // forward offset (distance from center to strafe wheel)
 * );
 * </pre>
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class ThreeWheelOdometry implements NovadOdometry {

    /**
     * Functional interface for reading encoder position
     */
    @FunctionalInterface
    public interface EncoderReader {
        int getPosition();
    }

    // Encoder readers
    private final EncoderReader leftEncoder;
    private final EncoderReader rightEncoder;
    private final EncoderReader centerEncoder;

    // Configuration
    private final double inchesPerTick;
    private final double trackWidth;
    private final double forwardOffset;

    // State
    private double x, y, heading;
    private double vx, vy, omega;

    // Previous encoder values
    private int prevLeftTicks, prevRightTicks, prevCenterTicks;
    private long prevTime;

    /**
     * Create a three-wheel odometry system
     * 
     * @param leftEncoder Left encoder position reader
     * @param rightEncoder Right encoder position reader
     * @param centerEncoder Center/strafe encoder position reader
     * @param wheelDiameter Wheel diameter in inches
     * @param ticksPerRevolution Encoder ticks per full wheel revolution
     * @param trackWidth Distance between left and right wheels in inches
     * @param forwardOffset Distance from robot center to center wheel in inches
     */
    public ThreeWheelOdometry(
            EncoderReader leftEncoder,
            EncoderReader rightEncoder,
            EncoderReader centerEncoder,
            double wheelDiameter,
            int ticksPerRevolution,
            double trackWidth,
            double forwardOffset) {
        
        this.leftEncoder = leftEncoder;
        this.rightEncoder = rightEncoder;
        this.centerEncoder = centerEncoder;

        // Calculate inches per encoder tick
        this.inchesPerTick = (wheelDiameter * Math.PI) / ticksPerRevolution;
        this.trackWidth = trackWidth;
        this.forwardOffset = forwardOffset;

        // Initialize state
        this.x = 0;
        this.y = 0;
        this.heading = 0;
        this.vx = 0;
        this.vy = 0;
        this.omega = 0;

        // Initialize previous values
        this.prevLeftTicks = leftEncoder.getPosition();
        this.prevRightTicks = rightEncoder.getPosition();
        this.prevCenterTicks = centerEncoder.getPosition();
        this.prevTime = System.nanoTime();
    }

    @Override
    public void update() {
        // Read current encoder positions
        int leftTicks = leftEncoder.getPosition();
        int rightTicks = rightEncoder.getPosition();
        int centerTicks = centerEncoder.getPosition();
        long currentTime = System.nanoTime();

        // Calculate deltas
        int deltaLeft = leftTicks - prevLeftTicks;
        int deltaRight = rightTicks - prevRightTicks;
        int deltaCenter = centerTicks - prevCenterTicks;
        double dt = (currentTime - prevTime) / 1_000_000_000.0;

        // Convert to inches
        double leftDist = deltaLeft * inchesPerTick;
        double rightDist = deltaRight * inchesPerTick;
        double centerDist = deltaCenter * inchesPerTick;

        // Calculate heading change
        double deltaHeading = (rightDist - leftDist) / trackWidth;

        // Calculate forward and strafe movement
        double deltaForward = (leftDist + rightDist) / 2.0;
        double deltaStrafe = centerDist - (forwardOffset * deltaHeading);

        // Update position using arc integration
        double avgHeading = heading + deltaHeading / 2.0;
        double cos = Math.cos(avgHeading);
        double sin = Math.sin(avgHeading);

        x += deltaForward * cos - deltaStrafe * sin;
        y += deltaForward * sin + deltaStrafe * cos;
        heading += deltaHeading;

        // Normalize heading to [-π, π]
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;

        // Calculate velocities
        if (dt > 0.001) {
            vx = (deltaForward * cos - deltaStrafe * sin) / dt;
            vy = (deltaForward * sin + deltaStrafe * cos) / dt;
            omega = deltaHeading / dt;
        }

        // Save current values
        prevLeftTicks = leftTicks;
        prevRightTicks = rightTicks;
        prevCenterTicks = centerTicks;
        prevTime = currentTime;
    }

    @Override
    public Vector2D getPosition() {
        return new Vector2D(x, y);
    }

    @Override
    public double getX() {
        return x;
    }

    @Override
    public double getY() {
        return y;
    }

    @Override
    public double getHeading() {
        return heading;
    }

    @Override
    public Vector2D getVelocity() {
        return new Vector2D(vx, vy);
    }

    @Override
    public double getXVelocity() {
        return vx;
    }

    @Override
    public double getYVelocity() {
        return vy;
    }

    @Override
    public double getAngularVelocity() {
        return omega;
    }

    @Override
    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
    }

    /**
     * Get the inches per encoder tick conversion factor
     * @return Inches per tick
     */
    public double getInchesPerTick() {
        return inchesPerTick;
    }

    /**
     * Get the track width
     * @return Track width in inches
     */
    public double getTrackWidth() {
        return trackWidth;
    }

    /**
     * Get the forward offset
     * @return Forward offset in inches
     */
    public double getForwardOffset() {
        return forwardOffset;
    }
}
