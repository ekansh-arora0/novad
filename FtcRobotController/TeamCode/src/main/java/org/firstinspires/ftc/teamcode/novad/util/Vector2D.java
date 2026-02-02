package org.firstinspires.ftc.teamcode.novad.util;

/**
 * Simple 2D vector math utility for Novad
 * Handles position, velocity, and force calculations
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class Vector2D {

    /** X component of the vector */
    public double x;

    /** Y component of the vector */
    public double y;

    /**
     * Create a zero vector
     */
    public Vector2D() {
        this.x = 0;
        this.y = 0;
    }

    /**
     * Create a vector with specified components
     * @param x X component
     * @param y Y component
     */
    public Vector2D(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Copy constructor
     * @param other Vector to copy
     */
    public Vector2D(Vector2D other) {
        this.x = other.x;
        this.y = other.y;
    }

    /**
     * Add another vector to this one
     * @param other Vector to add
     * @return New vector representing the sum
     */
    public Vector2D add(Vector2D other) {
        return new Vector2D(this.x + other.x, this.y + other.y);
    }

    /**
     * Subtract another vector from this one
     * @param other Vector to subtract
     * @return New vector representing the difference
     */
    public Vector2D subtract(Vector2D other) {
        return new Vector2D(this.x - other.x, this.y - other.y);
    }

    /**
     * Multiply vector by a scalar
     * @param scalar Value to multiply by
     * @return New scaled vector
     */
    public Vector2D multiply(double scalar) {
        return new Vector2D(this.x * scalar, this.y * scalar);
    }

    /**
     * Divide vector by a scalar
     * @param scalar Value to divide by
     * @return New scaled vector
     */
    public Vector2D divide(double scalar) {
        if (Math.abs(scalar) < 1e-10) {
            return new Vector2D(0, 0);
        }
        return new Vector2D(this.x / scalar, this.y / scalar);
    }

    /**
     * Get the magnitude (length) of this vector
     * @return Magnitude of the vector
     */
    public double magnitude() {
        return Math.sqrt(x * x + y * y);
    }

    /**
     * Get the squared magnitude (avoids sqrt for performance)
     * @return Squared magnitude
     */
    public double magnitudeSquared() {
        return x * x + y * y;
    }

    /**
     * Normalize this vector to unit length
     * @return New unit vector in same direction
     */
    public Vector2D normalize() {
        double mag = magnitude();
        if (mag < 1e-10) {
            return new Vector2D(0, 0);
        }
        return new Vector2D(x / mag, y / mag);
    }

    /**
     * Get the dot product with another vector
     * @param other Other vector
     * @return Dot product
     */
    public double dot(Vector2D other) {
        return this.x * other.x + this.y * other.y;
    }

    /**
     * Get the cross product (2D gives scalar)
     * @param other Other vector
     * @return Cross product (z-component)
     */
    public double cross(Vector2D other) {
        return this.x * other.y - this.y * other.x;
    }

    /**
     * Rotate this vector by an angle
     * @param angleRadians Angle in radians
     * @return New rotated vector
     */
    public Vector2D rotate(double angleRadians) {
        double cos = Math.cos(angleRadians);
        double sin = Math.sin(angleRadians);
        return new Vector2D(
            x * cos - y * sin,
            x * sin + y * cos
        );
    }

    /**
     * Get the angle of this vector from positive X axis
     * @return Angle in radians
     */
    public double angle() {
        return Math.atan2(y, x);
    }

    /**
     * Get distance to another point
     * @param other Other point
     * @return Distance between points
     */
    public double distanceTo(Vector2D other) {
        return this.subtract(other).magnitude();
    }

    /**
     * Clamp vector magnitude to a maximum value
     * @param maxMagnitude Maximum allowed magnitude
     * @return Clamped vector
     */
    public Vector2D clampMagnitude(double maxMagnitude) {
        double mag = magnitude();
        if (mag <= maxMagnitude) {
            return new Vector2D(this);
        }
        return this.normalize().multiply(maxMagnitude);
    }

    /**
     * Linear interpolation between this vector and another
     * @param target Target vector
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Interpolated vector
     */
    public Vector2D lerp(Vector2D target, double t) {
        t = Math.max(0, Math.min(1, t));
        return new Vector2D(
            this.x + (target.x - this.x) * t,
            this.y + (target.y - this.y) * t
        );
    }

    /**
     * Set components of this vector
     * @param x New X component
     * @param y New Y component
     */
    public void set(double x, double y) {
        this.x = x;
        this.y = y;
    }

    /**
     * Set components from another vector
     * @param other Vector to copy from
     */
    public void set(Vector2D other) {
        this.x = other.x;
        this.y = other.y;
    }

    /**
     * Check if this vector is approximately zero
     * @param epsilon Tolerance
     * @return True if magnitude is less than epsilon
     */
    public boolean isZero(double epsilon) {
        return magnitudeSquared() < epsilon * epsilon;
    }

    /**
     * Check if this vector is approximately equal to another
     * @param other Other vector
     * @param epsilon Tolerance
     * @return True if approximately equal
     */
    public boolean equals(Vector2D other, double epsilon) {
        return Math.abs(this.x - other.x) < epsilon && Math.abs(this.y - other.y) < epsilon;
    }

    @Override
    public String toString() {
        return String.format("(%.3f, %.3f)", x, y);
    }

    @Override
    public boolean equals(Object obj) {
        if (this == obj) return true;
        if (obj == null || getClass() != obj.getClass()) return false;
        Vector2D vector2D = (Vector2D) obj;
        return Double.compare(vector2D.x, x) == 0 && Double.compare(vector2D.y, y) == 0;
    }

    @Override
    public int hashCode() {
        long temp = Double.doubleToLongBits(x);
        int result = (int) (temp ^ (temp >>> 32));
        temp = Double.doubleToLongBits(y);
        result = 31 * result + (int) (temp ^ (temp >>> 32));
        return result;
    }

    // ========================================================================================
    // Static factory methods
    // ========================================================================================

    /**
     * Create a vector from polar coordinates
     * @param magnitude Length of vector
     * @param angleRadians Angle from positive X axis
     * @return New vector
     */
    public static Vector2D fromPolar(double magnitude, double angleRadians) {
        return new Vector2D(
            magnitude * Math.cos(angleRadians),
            magnitude * Math.sin(angleRadians)
        );
    }

    /**
     * Create a zero vector
     * @return New zero vector
     */
    public static Vector2D zero() {
        return new Vector2D(0, 0);
    }

    /**
     * Create a unit vector pointing right
     * @return Unit vector (1, 0)
     */
    public static Vector2D right() {
        return new Vector2D(1, 0);
    }

    /**
     * Create a unit vector pointing up
     * @return Unit vector (0, 1)
     */
    public static Vector2D up() {
        return new Vector2D(0, 1);
    }
}
