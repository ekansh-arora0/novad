package com.novad.tuning;

import com.novad.config.NovadConstants;
import com.novad.config.NovadConstants.DriveType;
import com.novad.config.NovadConstants.OdometryType;

/**
 * Generates intelligent starting PID values based on robot configuration
 * Provides recommended tuning profiles for different drivetrain and odometry combinations
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class ProfileGenerator {

    /**
     * Robot profile containing all PID values and settings
     */
    public static class RobotProfile {
        // Velocity PID
        public double velocityKP;
        public double velocityKI;
        public double velocityKD;

        // Position PID
        public double positionKP;
        public double positionKI;
        public double positionKD;

        // Heading PID
        public double headingKP;
        public double headingKI;
        public double headingKD;

        // Behavior settings
        public double activationDelayMs;
        public double rampTimeMs;
        public double joystickDeadband;
        public double correctionDeadband;
        public double maxCorrectionPower;

        // Features
        public boolean enableSlipDetection;

        /**
         * Create a profile with default values
         */
        public RobotProfile() {
            // Default velocity PID
            velocityKP = 1.5;
            velocityKI = 0.2;
            velocityKD = 0.1;

            // Default position PID
            positionKP = 0.8;
            positionKI = 0.15;
            positionKD = 0.05;

            // Default heading PID
            headingKP = 2.0;
            headingKI = 0.05;
            headingKD = 0.15;

            // Default behavior
            activationDelayMs = 75;
            rampTimeMs = 200;
            joystickDeadband = 0.08;
            correctionDeadband = 0.25;
            maxCorrectionPower = 0.85;

            // Features
            enableSlipDetection = false;
        }

        /**
         * Apply this profile to NovadConstants
         */
        public void applyToConstants() {
            NovadConstants.VELOCITY_KP = velocityKP;
            NovadConstants.VELOCITY_KI = velocityKI;
            NovadConstants.VELOCITY_KD = velocityKD;

            NovadConstants.POSITION_KP = positionKP;
            NovadConstants.POSITION_KI = positionKI;
            NovadConstants.POSITION_KD = positionKD;

            NovadConstants.HEADING_KP = headingKP;
            NovadConstants.HEADING_KI = headingKI;
            NovadConstants.HEADING_KD = headingKD;

            NovadConstants.ACTIVATION_DELAY_MS = activationDelayMs;
            NovadConstants.RAMP_TIME_MS = rampTimeMs;
            NovadConstants.JOYSTICK_DEADBAND = joystickDeadband;
            NovadConstants.CORRECTION_DEADBAND = correctionDeadband;
            NovadConstants.MAX_CORRECTION_POWER = maxCorrectionPower;

            NovadConstants.ENABLE_SLIP_DETECTION = enableSlipDetection;
        }

        @Override
        public String toString() {
            StringBuilder sb = new StringBuilder();
            sb.append("=== Velocity PID ===\n");
            sb.append(String.format("  kP: %.3f\n", velocityKP));
            sb.append(String.format("  kI: %.3f\n", velocityKI));
            sb.append(String.format("  kD: %.3f\n", velocityKD));
            sb.append("\n=== Position PID ===\n");
            sb.append(String.format("  kP: %.3f\n", positionKP));
            sb.append(String.format("  kI: %.3f\n", positionKI));
            sb.append(String.format("  kD: %.3f\n", positionKD));
            sb.append("\n=== Heading PID ===\n");
            sb.append(String.format("  kP: %.3f\n", headingKP));
            sb.append(String.format("  kI: %.3f\n", headingKI));
            sb.append(String.format("  kD: %.3f\n", headingKD));
            sb.append("\n=== Behavior ===\n");
            sb.append(String.format("  Delay: %.0f ms\n", activationDelayMs));
            sb.append(String.format("  Ramp: %.0f ms\n", rampTimeMs));
            sb.append(String.format("  Joystick Deadband: %.3f\n", joystickDeadband));
            sb.append(String.format("  Correction Deadband: %.3f in\n", correctionDeadband));
            sb.append(String.format("  Max Power: %.2f\n", maxCorrectionPower));
            sb.append(String.format("  Slip Detection: %s\n", enableSlipDetection));
            return sb.toString();
        }
    }

    /**
     * Generate a profile for the configured robot
     * Uses DriveType and OdometryType from NovadConstants
     * @return Optimized profile for your robot
     */
    public static RobotProfile generateProfile() {
        return generateProfile(NovadConstants.DRIVE_TYPE, NovadConstants.ODOMETRY_TYPE);
    }

    /**
     * Generate a profile for a specific drive and odometry combination
     * @param driveType Your drivetrain type
     * @param odometryType Your odometry type
     * @return Optimized profile
     */
    public static RobotProfile generateProfile(DriveType driveType, OdometryType odometryType) {
        RobotProfile profile = new RobotProfile();

        // Apply drive type modifiers
        applyDriveModifiers(profile, driveType);

        // Apply odometry type modifiers
        applyOdometryModifiers(profile, odometryType);

        return profile;
    }

    /**
     * Apply modifiers based on drivetrain type
     */
    private static void applyDriveModifiers(RobotProfile profile, DriveType driveType) {
        switch (driveType) {
            case MECANUM_4_MOTOR:
                // Baseline - no changes
                break;

            case MECANUM_6_MOTOR:
                // More power available - increase gains, reduce ramp time
                profile.velocityKP *= 1.15;
                profile.velocityKI *= 1.15;
                profile.velocityKD *= 1.15;
                profile.positionKP *= 1.15;
                profile.positionKI *= 1.15;
                profile.positionKD *= 1.15;
                profile.headingKP *= 1.15;
                profile.headingKI *= 1.15;
                profile.headingKD *= 1.15;
                profile.rampTimeMs *= 0.85;
                profile.maxCorrectionPower = 0.9;
                break;

            case TANK:
                // Simpler dynamics, less agile, stronger heading
                profile.headingKP *= 1.25;
                profile.headingKI *= 1.25;
                profile.headingKD *= 1.25;
                profile.positionKP *= 0.9;
                profile.positionKI *= 0.9;
                // Tank can't strafe - zero out strafe corrections
                profile.velocityKP *= 0.8;
                break;

            case H_DRIVE:
                // Independent strafe capability
                profile.velocityKP *= 1.1;
                profile.velocityKI *= 1.1;
                profile.positionKI *= 1.2;
                break;
        }
    }

    /**
     * Apply modifiers based on odometry type
     */
    private static void applyOdometryModifiers(RobotProfile profile, OdometryType odometryType) {
        switch (odometryType) {
            case THREE_WHEEL:
                // Most accurate - tighten deadbands, enable slip detection
                profile.correctionDeadband *= 0.9;
                profile.joystickDeadband *= 0.9;
                profile.enableSlipDetection = true;
                break;

            case TWO_WHEEL_IMU:
                // IMU drift compensation needed
                profile.velocityKI *= 1.15;
                profile.positionKI *= 1.2;
                profile.headingKP *= 1.1;
                profile.headingKI *= 1.15;
                profile.correctionDeadband *= 1.1;
                profile.joystickDeadband *= 1.1;
                break;

            case PEDRO_PATHING:
                // Trust the localization - tighten deadbands
                profile.correctionDeadband *= 0.85;
                profile.joystickDeadband *= 0.85;
                profile.velocityKP *= 1.05;
                profile.enableSlipDetection = true;
                break;

            case ROADRUNNER:
                // Very accurate - tighten deadbands, enable slip detection
                profile.correctionDeadband *= 0.85;
                profile.joystickDeadband *= 0.85;
                profile.enableSlipDetection = true;
                break;
        }
    }

    /**
     * Get a description of the generated profile
     */
    public static String getProfileDescription(DriveType driveType, OdometryType odometryType) {
        StringBuilder sb = new StringBuilder();
        sb.append("Profile for: ").append(driveType).append(" + ").append(odometryType).append("\n\n");

        switch (driveType) {
            case MECANUM_4_MOTOR:
                sb.append("Drive: Standard 4-motor mecanum (baseline)\n");
                break;
            case MECANUM_6_MOTOR:
                sb.append("Drive: 6-motor mecanum (gains +15%, ramp -15%)\n");
                break;
            case TANK:
                sb.append("Drive: Tank/differential (heading +25%, position -10%)\n");
                break;
            case H_DRIVE:
                sb.append("Drive: H-drive (velocity +10%, integral +20%)\n");
                break;
        }

        switch (odometryType) {
            case THREE_WHEEL:
                sb.append("Odometry: Three-wheel (deadbands -10%, slip detection ON)\n");
                break;
            case TWO_WHEEL_IMU:
                sb.append("Odometry: Two-wheel + IMU (integrals +15-20%, deadbands +10%)\n");
                break;
            case PEDRO_PATHING:
                sb.append("Odometry: Pedro Pathing (deadbands -15%, velocity +5%)\n");
                break;
            case ROADRUNNER:
                sb.append("Odometry: RoadRunner (deadbands -15%, slip detection ON)\n");
                break;
        }

        return sb.toString();
    }

    /**
     * Generate Java code to copy into NovadConstants
     */
    public static String generateConstantsCode(RobotProfile profile) {
        StringBuilder sb = new StringBuilder();
        sb.append("// ===== Generated by ProfileGenerator =====\n");
        sb.append("// Copy these values to your NovadConstants.java\n\n");

        sb.append("// Velocity PID\n");
        sb.append(String.format("public static double VELOCITY_KP = %.4f;\n", profile.velocityKP));
        sb.append(String.format("public static double VELOCITY_KI = %.4f;\n", profile.velocityKI));
        sb.append(String.format("public static double VELOCITY_KD = %.4f;\n", profile.velocityKD));
        sb.append("\n");

        sb.append("// Position PID\n");
        sb.append(String.format("public static double POSITION_KP = %.4f;\n", profile.positionKP));
        sb.append(String.format("public static double POSITION_KI = %.4f;\n", profile.positionKI));
        sb.append(String.format("public static double POSITION_KD = %.4f;\n", profile.positionKD));
        sb.append("\n");

        sb.append("// Heading PID\n");
        sb.append(String.format("public static double HEADING_KP = %.4f;\n", profile.headingKP));
        sb.append(String.format("public static double HEADING_KI = %.4f;\n", profile.headingKI));
        sb.append(String.format("public static double HEADING_KD = %.4f;\n", profile.headingKD));
        sb.append("\n");

        sb.append("// Behavior Settings\n");
        sb.append(String.format("public static double ACTIVATION_DELAY_MS = %.1f;\n", profile.activationDelayMs));
        sb.append(String.format("public static double RAMP_TIME_MS = %.1f;\n", profile.rampTimeMs));
        sb.append(String.format("public static double JOYSTICK_DEADBAND = %.4f;\n", profile.joystickDeadband));
        sb.append(String.format("public static double CORRECTION_DEADBAND = %.4f;\n", profile.correctionDeadband));
        sb.append(String.format("public static double MAX_CORRECTION_POWER = %.2f;\n", profile.maxCorrectionPower));
        sb.append(String.format("public static boolean ENABLE_SLIP_DETECTION = %s;\n", profile.enableSlipDetection));

        return sb.toString();
    }
}
