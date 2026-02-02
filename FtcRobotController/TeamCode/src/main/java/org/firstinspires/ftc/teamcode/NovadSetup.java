package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════════╗
 * ║                           NOVAD SETUP                                         ║
 * ║                                                                               ║
 * ║  Fill in YOUR robot's values below, then you're done!                         ║
 * ║  All values can be tuned live in FTC Dashboard.                               ║
 * ╚═══════════════════════════════════════════════════════════════════════════════╝
 * 
 * STEP 1: Open your Robot Configuration on the Driver Station
 * STEP 2: Find your motor names and type them below (in quotes)
 * STEP 3: Deploy to robot
 * STEP 4: Run "Novad TeleOp" from the Driver Station
 * 
 * That's it! If your robot drives backwards or wrong, flip the REVERSED values.
 */
@Config  // This makes all values editable in FTC Dashboard!
public class NovadSetup {

    // ╔═══════════════════════════════════════════════════════════════════════════╗
    // ║  STEP 1: YOUR MOTOR NAMES                                                 ║
    // ║  Copy these EXACTLY from your Robot Configuration                         ║
    // ╚═══════════════════════════════════════════════════════════════════════════╝
    
    public static String FRONT_LEFT  = "frontLeft";   // ← Change this
    public static String FRONT_RIGHT = "frontRight";  // ← Change this
    public static String BACK_LEFT   = "backLeft";    // ← Change this
    public static String BACK_RIGHT  = "backRight";   // ← Change this

    // ╔═══════════════════════════════════════════════════════════════════════════╗
    // ║  STEP 2: MOTOR DIRECTIONS                                                 ║
    // ║  If a motor runs backwards, change its value to true                      ║
    // ╚═══════════════════════════════════════════════════════════════════════════╝
    
    public static boolean FRONT_LEFT_REVERSED  = true;   // Typical: left side reversed
    public static boolean FRONT_RIGHT_REVERSED = false;
    public static boolean BACK_LEFT_REVERSED   = true;   // Typical: left side reversed
    public static boolean BACK_RIGHT_REVERSED  = false;

    // ╔═══════════════════════════════════════════════════════════════════════════╗
    // ║  STEP 3: ODOMETRY TYPE                                                    ║
    // ║  Pick ONE by setting it to true, others to false                          ║
    // ╚═══════════════════════════════════════════════════════════════════════════╝
    
    public static boolean USE_PINPOINT    = true;   // GoBilda Pinpoint (recommended)
    public static boolean USE_THREE_WHEEL = false;  // Three dead wheel encoders

    // ─────────────────────────────────────────────────────────────────────────────
    // PINPOINT SETTINGS (only if USE_PINPOINT = true)
    // ─────────────────────────────────────────────────────────────────────────────
    
    public static String PINPOINT_NAME = "pinpoint";  // Name in Robot Config (I2C device)

    // ─────────────────────────────────────────────────────────────────────────────
    // THREE WHEEL SETTINGS (only if USE_THREE_WHEEL = true)
    // Dead wheel encoders plug into MOTOR ENCODER PORTS
    // Write the name of the motor whose encoder port you're using
    // ─────────────────────────────────────────────────────────────────────────────
    
    public static String LEFT_ENCODER_PORT   = "frontLeft";   // Motor with left encoder
    public static String RIGHT_ENCODER_PORT  = "frontRight";  // Motor with right encoder  
    public static String CENTER_ENCODER_PORT = "backLeft";    // Motor with center encoder
    
    public static boolean LEFT_ENCODER_REVERSED   = false;
    public static boolean RIGHT_ENCODER_REVERSED  = false;
    public static boolean CENTER_ENCODER_REVERSED = false;
    
    public static double WHEEL_DIAMETER_INCHES = 1.89;  // 48mm pods = 1.89 inches
    public static double TRACK_WIDTH_INCHES    = 14.0;  // Distance between L/R wheels
    public static double FORWARD_OFFSET_INCHES = 6.0;   // Center wheel offset from robot center

    // ╔═══════════════════════════════════════════════════════════════════════════╗
    // ║  TUNING VALUES - Adjust these in FTC Dashboard while running!             ║
    // ║  Start with defaults, then tune if needed                                 ║
    // ╚═══════════════════════════════════════════════════════════════════════════╝
    
    // Position hold (how hard it resists being pushed sideways/forward)
    public static double POSITION_P = 0.046;
    public static double POSITION_I = 0.0;
    public static double POSITION_D = 0.01;
    
    // Heading hold (how hard it resists being rotated)
    public static double HEADING_P = 0.67;
    public static double HEADING_I = 0.0;
    public static double HEADING_D = 0.006;
    
    // ╔═══════════════════════════════════════════════════════════════════════════╗
    // ║  PREDICTIVE DEFENSE - Makes Novad respond faster!                         ║
    // ║  Leave these at defaults unless you want to experiment                    ║
    // ╚═══════════════════════════════════════════════════════════════════════════╝
    
    public static boolean PREDICTIVE_ENABLED = true;    // Predict pushes before they happen
    public static double PREDICTION_MS       = 50;      // How far ahead to predict (ms)
    public static double BOOST_MULTIPLIER    = 1.5;     // Extra power when impact detected

    // ╔═══════════════════════════════════════════════════════════════════════════╗
    // ║  ADVANCED - You probably don't need to change these                       ║
    // ╚═══════════════════════════════════════════════════════════════════════════╝
    
    public static double MAX_POWER      = 0.8;   // Max correction power (0-1)
    public static double DEAD_ZONE      = 0.1;   // Joystick deadzone
    public static double RAMP_TIME      = 0.3;   // Seconds to reach full power
}
