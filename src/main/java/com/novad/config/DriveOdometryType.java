package com.novad.config;

/**
 * Drive and Odometry type enums used by ProfileGenerator and other helpers
 */
public final class DriveOdometryType {

    public enum DriveType {
        MECANUM_4_MOTOR,
        MECANUM_6_MOTOR,
        TANK,
        H_DRIVE
    }

    public enum OdometryType {
        THREE_WHEEL,
        TWO_WHEEL_IMU,
        CUSTOM
    }

    private DriveOdometryType() {}
}
