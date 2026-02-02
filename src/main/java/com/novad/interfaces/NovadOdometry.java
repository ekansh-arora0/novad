package com.novad.interfaces;

import com.novad.util.Vector2D;

/**
 * Interface for odometry systems that Novad can use
 * Implement this interface to connect Novad to your odometry
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public interface NovadOdometry {

    /**
     * Get the current robot position in field coordinates
     * @return Position vector (x, y) in inches
     */
    Vector2D getPosition();

    /**
     * Get the current X position
     * @return X position in inches
     */
    double getX();

    /**
     * Get the current Y position
     * @return Y position in inches
     */
    double getY();

    /**
     * Get the current robot heading
     * @return Heading in radians (positive = counter-clockwise)
     */
    double getHeading();

    /**
     * Get the current robot velocity
     * @return Velocity vector (vx, vy) in inches per second
     */
    Vector2D getVelocity();

    /**
     * Get the current X velocity
     * @return X velocity in inches per second
     */
    double getXVelocity();

    /**
     * Get the current Y velocity
     * @return Y velocity in inches per second
     */
    double getYVelocity();

    /**
     * Get the current angular velocity
     * @return Angular velocity in radians per second
     */
    double getAngularVelocity();

    /**
     * Update the odometry readings
     * Call this every loop iteration before using position/velocity data
     */
    void update();

    /**
     * Set the robot pose (position and heading)
     * Used for field-relative initialization or correction
     * @param x X position in inches
     * @param y Y position in inches
     * @param heading Heading in radians
     */
    void setPose(double x, double y, double heading);

    /**
     * Reset position to zero
     */
    default void resetPosition() {
        setPose(0, 0, getHeading());
    }

    /**
     * Reset heading to zero
     */
    default void resetHeading() {
        setPose(getX(), getY(), 0);
    }

    /**
     * Reset everything to zero
     */
    default void reset() {
        setPose(0, 0, 0);
    }
}
