package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * ═══════════════════════════════════════════════════════════════════════════════
 * NOVAD CONSTANTS - Configure ALL your robot settings here!
 * ═══════════════════════════════════════════════════════════════════════════════
 * 
 * This is the ONLY file you need to edit to configure Novad.
 * All values are tunable via FTC Dashboard (panels.bylazar.com or 192.168.43.1:8080/dash)
 * 
 * SETUP STEPS:
 * 1. Set your motor names (from Robot Configuration)
 * 2. Set motor directions (test which side needs REVERSE)
 * 3. Choose your odometry type (PINPOINT or THREE_WHEEL)
 * 4. Set odometry parameters
 * 5. Run robot and tune PIDF values in Dashboard
 */
@Config
public class NovadConstants {

    // ═══════════════════════════════════════════════════════════════════════════
    // MOTOR NAMES - Change these to match your Robot Configuration!
    // ═══════════════════════════════════════════════════════════════════════════
    
    public static String LEFT_FRONT_MOTOR = "frontLeft";
    public static String LEFT_REAR_MOTOR = "backLeft";
    public static String RIGHT_FRONT_MOTOR = "frontRight";
    public static String RIGHT_REAR_MOTOR = "backRight";
    
    // ═══════════════════════════════════════════════════════════════════════════
    // MOTOR DIRECTIONS - Set to true if motor runs backwards
    // Typical mecanum: left side reversed, right side forward
    // ═══════════════════════════════════════════════════════════════════════════
    
    public static boolean LEFT_FRONT_REVERSED = true;
    public static boolean LEFT_REAR_REVERSED = true;
    public static boolean RIGHT_FRONT_REVERSED = false;
    public static boolean RIGHT_REAR_REVERSED = false;

    // ═══════════════════════════════════════════════════════════════════════════
    // ODOMETRY TYPE - Choose your localization method
    // ═══════════════════════════════════════════════════════════════════════════
    
    public enum OdometryType {
        PINPOINT,      // GoBilda Pinpoint (recommended)
        THREE_WHEEL,   // Three dead wheel encoders
        DRIVE_ENCODERS // Use drive motor encoders (least accurate)
    }
    
    public static OdometryType ODOMETRY_TYPE = OdometryType.PINPOINT;

    // ═══════════════════════════════════════════════════════════════════════════
    // PINPOINT ODOMETRY CONFIG (if using GoBilda Pinpoint)
    // ═══════════════════════════════════════════════════════════════════════════
    
    /** I2C device name in robot configuration */
    public static String PINPOINT_DEVICE_NAME = "pinpoint";
    
    /** X offset from robot center to Pinpoint in mm (positive = forward) */
    public static double PINPOINT_X_OFFSET_MM = 0;
    
    /** Y offset from robot center to Pinpoint in mm (positive = left) */
    public static double PINPOINT_Y_OFFSET_MM = 0;
    
    /** Encoder resolution - use GoBilda constant or calculate */
    public static double PINPOINT_ENCODER_RESOLUTION = 8192; // GoBilda goBILDA® 4-Bar Odometry Pod
    
    /** Pod wheel diameter in mm */
    public static double PINPOINT_WHEEL_DIAMETER_MM = 48; // Standard GoBilda odometry pod
    
    // ═══════════════════════════════════════════════════════════════════════════
    // THREE WHEEL ODOMETRY CONFIG (if using dead wheels)
    // Encoders are plugged into motor encoder ports!
    // ═══════════════════════════════════════════════════════════════════════════
    
    /** Motor port with left encoder plugged into its encoder port */
    public static String LEFT_ENCODER_MOTOR_PORT = "frontLeft";
    
    /** Motor port with right encoder plugged into its encoder port */
    public static String RIGHT_ENCODER_MOTOR_PORT = "frontRight";
    
    /** Motor port with center/strafe encoder plugged into its encoder port */
    public static String CENTER_ENCODER_MOTOR_PORT = "backLeft";
    
    /** Set to true if encoder direction is reversed */
    public static boolean LEFT_ENCODER_REVERSED = false;
    public static boolean RIGHT_ENCODER_REVERSED = false;
    public static boolean CENTER_ENCODER_REVERSED = false;
    
    /** Wheel diameter in inches (48mm ≈ 1.89in) */
    public static double DEAD_WHEEL_DIAMETER_INCHES = 1.89;
    
    /** Encoder ticks per revolution */
    public static int DEAD_WHEEL_TICKS_PER_REV = 8192;
    
    /** Distance between left and right wheels in inches */
    public static double TRACK_WIDTH_INCHES = 14.0;
    
    /** Forward offset: distance from center to strafe wheel (positive = forward) */
    public static double FORWARD_OFFSET_INCHES = 6.0;

    // ═══════════════════════════════════════════════════════════════════════════
    // TRANSLATIONAL (POSITION) PID - Tune these for X/Y position hold
    // Start with P, then add D if oscillating. Usually keep I at 0.
    // ═══════════════════════════════════════════════════════════════════════════
    
    public static double TRANS_P = 0.046;
    public static double TRANS_I = 0.0;
    public static double TRANS_D = 0.01;
    public static double TRANS_F = 0.02;

    // ═══════════════════════════════════════════════════════════════════════════
    // HEADING (ROTATION) PID - Tune these for heading hold
    // Usually needs higher P than translational
    // ═══════════════════════════════════════════════════════════════════════════
    
    public static double HEADING_P = 0.67;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.006;
    public static double HEADING_F = 0.02;

    // ═══════════════════════════════════════════════════════════════════════════
    // VELOCITY PID - Tune these for velocity-based corrections
    // These provide the "instant response" layer
    // ═══════════════════════════════════════════════════════════════════════════
    
    public static double VEL_P = 0.02;
    public static double VEL_I = 0.0;
    public static double VEL_D = 0.001;
    public static double VEL_F = 0.01;

    // ═══════════════════════════════════════════════════════════════════════════
    // PREDICTIVE DEFENSE (NEW!) - Faster response without sacrificing accuracy
    // Uses velocity + acceleration to predict where you'll be pushed
    // ═══════════════════════════════════════════════════════════════════════════
    
    /** Enable predictive defense for faster response */
    public static boolean PREDICTIVE_ENABLED = true;
    
    /** How many ms ahead to predict (higher = faster response, but less stable) */
    public static double PREDICTION_LOOKAHEAD_MS = 50;
    
    /** Acceleration threshold to trigger instant response (in/s²) */
    public static double ACCEL_TRIGGER_THRESHOLD = 20.0;
    
    /** Instant boost multiplier when sudden acceleration detected (1.0 = no boost) */
    public static double INSTANT_BOOST_MULTIPLIER = 1.5;

    // ═══════════════════════════════════════════════════════════════════════════
    // THRESHOLDS & LIMITS
    // ═══════════════════════════════════════════════════════════════════════════
    
    /** Minimum movement (inches) before defense activates */
    public static double MOVEMENT_THRESHOLD = 0.5;
    
    /** Minimum rotation (degrees) before defense activates */
    public static double HEADING_THRESHOLD_DEGREES = 2.0;
    
    /** Maximum motor power for corrections (0.0 to 1.0) */
    public static double MAX_CORRECTION_POWER = 0.8;
    
    /** Joystick deadzone - below this, driver is considered "not moving" */
    public static double DRIVER_DEADZONE = 0.1;
    
    /** Ramp-up time in seconds (prevents jerky initial corrections) */
    public static double RAMP_UP_TIME = 0.3;
    
    /** Anti-windup: max integral accumulation */
    public static double MAX_INTEGRAL = 1.0;
}
