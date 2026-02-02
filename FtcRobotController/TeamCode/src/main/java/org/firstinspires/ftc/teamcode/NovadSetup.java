package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

/**
 * ╔═══════════════════════════════════════════════════════════════════════════╗
 * ║                           NOVAD CONFIGURATION                              ║
 * ║                     The Knight's Defense System                            ║
 * ╚═══════════════════════════════════════════════════════════════════════════╝
 *
 * This is the ONLY file you need to edit to set up Novad.
 * All your robot's hardware names, motor directions, and tuning values go here.
 *
 * STEP-BY-STEP SETUP:
 * 1. Set your motor names (must match your Robot Configuration EXACTLY)
 * 2. Set motor directions (flip FORWARD↔REVERSE if a wheel spins backward)
 * 3. Choose your odometry: Pinpoint or Three-Wheel
 * 4. If Pinpoint: just set the device name
 *    If Three-Wheel: set encoder ports and physical measurements
 * 5. Deploy and test - tune PID values if needed
 *
 * All @Config values can be tuned LIVE in FTC Dashboard:
 * → Connect to robot WiFi
 * → Open http://192.168.43.1:8080/dash
 * → Expand "NovadSetup" section
 * → Change values and see instant results
 */
@Config
public class NovadSetup {

    // ═══════════════════════════════════════════════════════════════════════════
    //                        DRIVE MOTOR NAMES
    // ═══════════════════════════════════════════════════════════════════════════
    // These MUST match your Driver Station robot configuration exactly.
    // Go to Configure Robot → Edit → find your motor names

    public static String FRONT_LEFT  = "frontLeft";
    public static String FRONT_RIGHT = "frontRight";
    public static String BACK_LEFT   = "backLeft";
    public static String BACK_RIGHT  = "backRight";

    // ═══════════════════════════════════════════════════════════════════════════
    //                        MOTOR DIRECTIONS
    // ═══════════════════════════════════════════════════════════════════════════
    // Standard mecanum: left side = REVERSE, right side = FORWARD
    // If your robot drives backward when you push forward, flip all of them.
    // If just ONE wheel spins wrong, flip only that one.

    public static DcMotorSimple.Direction FRONT_LEFT_DIR  = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction FRONT_RIGHT_DIR = DcMotorSimple.Direction.FORWARD;
    public static DcMotorSimple.Direction BACK_LEFT_DIR   = DcMotorSimple.Direction.REVERSE;
    public static DcMotorSimple.Direction BACK_RIGHT_DIR  = DcMotorSimple.Direction.FORWARD;

    // ═══════════════════════════════════════════════════════════════════════════
    //                     ODOMETRY TYPE SELECTION
    // ═══════════════════════════════════════════════════════════════════════════
    // Pick ONE. Set your choice to TRUE, the other to FALSE.

    /** GoBilda Pinpoint Odometry Computer (recommended - easiest to set up) */
    public static boolean USE_PINPOINT = true;

    /** Three dead wheel encoders (requires more configuration below) */
    public static boolean USE_THREE_WHEEL = false;

    // ═══════════════════════════════════════════════════════════════════════════
    //                    PINPOINT CONFIGURATION
    //                (only fill this if USE_PINPOINT = true)
    // ═══════════════════════════════════════════════════════════════════════════

    /** Device name in your robot configuration */
    public static String PINPOINT_NAME = "pinpoint";

    // ═══════════════════════════════════════════════════════════════════════════
    //                  THREE-WHEEL ODOMETRY CONFIGURATION
    //               (only fill this if USE_THREE_WHEEL = true)
    // ═══════════════════════════════════════════════════════════════════════════
    //
    // Dead wheel encoders plug into MOTOR ENCODER PORTS.
    // Enter the name of the MOTOR whose port each encoder uses.

    public static String LEFT_ENCODER   = "frontLeft";
    public static String RIGHT_ENCODER  = "frontRight";
    public static String CENTER_ENCODER = "backLeft";

    /** Flip direction: set TRUE if encoder counts the wrong way */
    public static boolean LEFT_ENCODER_REVERSED   = false;
    public static boolean RIGHT_ENCODER_REVERSED  = false;
    public static boolean CENTER_ENCODER_REVERSED = false;

    /**
     * TICKS TO INCHES - CRITICAL VALUE!
     *
     * This converts encoder counts to real-world inches.
     * Formula: (wheel circumference in inches) / (encoder ticks per revolution)
     *
     * goBilda 48mm + REV Through Bore (8192 CPR):
     *   π × 1.89 / 8192 = 0.000724
     *
     * goBilda 48mm + REV Through Bore (2000 CPR Quadrature):
     *   π × 1.89 / 2000 = 0.00297
     */
    public static double TICKS_TO_INCHES = 0.000724;

    /** Distance between left and right encoders in inches (center to center) */
    public static double TRACK_WIDTH = 14.0;

    /** Distance from robot center to center encoder (+ = forward, - = backward) */
    public static double CENTER_OFFSET = 6.0;

    // ═══════════════════════════════════════════════════════════════════════════
    //                        PID TUNING VALUES
    // ═══════════════════════════════════════════════════════════════════════════
    //
    // These control how hard Novad resists being pushed.
    // Tune LIVE in FTC Dashboard, then copy final values here.
    //
    // TUNING GUIDE:
    // • Robot oscillates (shakes) → decrease P, increase D
    // • Robot doesn't hold position → increase P
    // • Robot drifts slowly over time → add small I value
    //
    // ─────── TRANSLATIONAL (X/Y position hold) ───────

    public static double TRANS_P = 0.1;    // Proportional: main correction strength
    public static double TRANS_I = 0.0;    // Integral: fixes steady-state drift (usually 0)
    public static double TRANS_D = 0.01;   // Derivative: dampens oscillation

    // ─────── HEADING (rotation hold) ───────

    public static double HEADING_P = 1.0;  // Proportional
    public static double HEADING_I = 0.0;  // Integral (usually 0)
    public static double HEADING_D = 0.01; // Derivative

    // ═══════════════════════════════════════════════════════════════════════════
    //                        ADVANCED SETTINGS
    // ═══════════════════════════════════════════════════════════════════════════

    /** Maximum correction power (0.0 to 1.0) */
    public static double MAX_POWER = 0.8;

    /** Joystick deadzone - values below this are treated as 0 */
    public static double DEADZONE = 0.05;

    /** Use field-centric driving (requires heading from odometry) */
    public static boolean USE_FIELD_CENTRIC = false;

    /** Use brake mode on motors when stopped */
    public static boolean USE_BRAKE_MODE = true;
}
