package com.novad.util;

/**
 * Math utilities for Novad defense system
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class MathUtils {

    /**
     * Normalize an angle to the range [-π, π]
     * @param angle Angle in radians
     * @return Normalized angle
     */
    public static double normalizeAngle(double angle) {
        while (angle > Math.PI) {
            angle -= 2 * Math.PI;
        }
        while (angle < -Math.PI) {
            angle += 2 * Math.PI;
        }
        return angle;
    }

    /**
     * Normalize an angle to the range [-180, 180]
     * @param angle Angle in degrees
     * @return Normalized angle
     */
    public static double normalizeAngleDegrees(double angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }

    /**
     * Convert degrees to radians
     * @param degrees Angle in degrees
     * @return Angle in radians
     */
    public static double toRadians(double degrees) {
        return degrees * Math.PI / 180.0;
    }

    /**
     * Convert radians to degrees
     * @param radians Angle in radians
     * @return Angle in degrees
     */
    public static double toDegrees(double radians) {
        return radians * 180.0 / Math.PI;
    }

    /**
     * Clamp a value between min and max
     * @param value Value to clamp
     * @param min Minimum value
     * @param max Maximum value
     * @return Clamped value
     */
    public static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }

    /**
     * Clamp a value to [-limit, +limit]
     * @param value Value to clamp
     * @param limit Absolute limit
     * @return Clamped value
     */
    public static double clampSymmetric(double value, double limit) {
        return clamp(value, -Math.abs(limit), Math.abs(limit));
    }

    /**
     * Linear interpolation between two values
     * @param a Start value
     * @param b End value
     * @param t Interpolation factor (0.0 to 1.0)
     * @return Interpolated value
     */
    public static double lerp(double a, double b, double t) {
        t = clamp(t, 0, 1);
        return a + (b - a) * t;
    }

    /**
     * Inverse linear interpolation - find t where lerp(a, b, t) = value
     * @param a Start value
     * @param b End value
     * @param value Value to find t for
     * @return Interpolation factor
     */
    public static double inverseLerp(double a, double b, double value) {
        if (Math.abs(b - a) < 1e-10) {
            return 0;
        }
        return (value - a) / (b - a);
    }

    /**
     * Check if a value is within a deadband of zero
     * @param value Value to check
     * @param deadband Deadband threshold
     * @return True if |value| < deadband
     */
    public static boolean inDeadband(double value, double deadband) {
        return Math.abs(value) < deadband;
    }

    /**
     * Apply deadband to a value
     * Returns 0 if value is within deadband, otherwise returns value
     * @param value Input value
     * @param deadband Deadband threshold
     * @return Value with deadband applied
     */
    public static double applyDeadband(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        return value;
    }

    /**
     * Apply deadband with scaling - removes deadband and scales remaining range
     * @param value Input value
     * @param deadband Deadband threshold
     * @return Scaled value (0 at deadband, 1 at full stick)
     */
    public static double applyDeadbandScaled(double value, double deadband) {
        if (Math.abs(value) < deadband) {
            return 0;
        }
        double sign = Math.signum(value);
        double magnitude = Math.abs(value);
        return sign * (magnitude - deadband) / (1.0 - deadband);
    }

    /**
     * Calculate the shortest angle difference between two angles
     * @param from Starting angle in radians
     * @param to Target angle in radians
     * @return Shortest signed angle difference
     */
    public static double angleDifference(double from, double to) {
        return normalizeAngle(to - from);
    }

    /**
     * Smooth step function (ease in/out)
     * @param t Input value (0.0 to 1.0)
     * @return Smoothed value
     */
    public static double smoothStep(double t) {
        t = clamp(t, 0, 1);
        return t * t * (3 - 2 * t);
    }

    /**
     * Smoother step function (higher order polynomial)
     * @param t Input value (0.0 to 1.0)
     * @return Smoothed value
     */
    public static double smootherStep(double t) {
        t = clamp(t, 0, 1);
        return t * t * t * (t * (t * 6 - 15) + 10);
    }

    /**
     * Check if two doubles are approximately equal
     * @param a First value
     * @param b Second value
     * @param epsilon Tolerance
     * @return True if approximately equal
     */
    public static boolean epsilonEquals(double a, double b, double epsilon) {
        return Math.abs(a - b) < epsilon;
    }

    /**
     * Sign function that returns 1, -1, or 0
     * @param value Input value
     * @return Sign of value
     */
    public static double sign(double value) {
        if (value > 0) return 1;
        if (value < 0) return -1;
        return 0;
    }

    /**
     * Calculate exponential moving average
     * @param current Current EMA value
     * @param newValue New sample
     * @param alpha Smoothing factor (0-1, higher = more responsive)
     * @return Updated EMA
     */
    public static double ema(double current, double newValue, double alpha) {
        return alpha * newValue + (1 - alpha) * current;
    }

    /**
     * Remap a value from one range to another
     * @param value Value to remap
     * @param fromMin Source range minimum
     * @param fromMax Source range maximum
     * @param toMin Target range minimum
     * @param toMax Target range maximum
     * @return Remapped value
     */
    public static double remap(double value, double fromMin, double fromMax, double toMin, double toMax) {
        double t = inverseLerp(fromMin, fromMax, value);
        return lerp(toMin, toMax, t);
    }

    /**
     * Calculate hypotenuse without overflow
     * @param x X component
     * @param y Y component
     * @return sqrt(x^2 + y^2)
     */
    public static double hypot(double x, double y) {
        return Math.hypot(x, y);
    }
}
