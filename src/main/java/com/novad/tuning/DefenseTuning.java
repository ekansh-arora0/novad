package com.novad.tuning;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;

import com.novad.Novad;
import com.novad.config.NovadConstants;
import com.novad.interfaces.NovadDrivetrain;
import com.novad.interfaces.NovadOdometry;
import com.novad.util.Vector2D;

/**
 * Defense Tuning OpMode for Novad
 * 
 * HOW TO USE:
 * 1. Deploy this OpMode to your robot
 * 2. Run it from the Driver Station
 * 3. Open panels.bylazar.com or go to http://192.168.43.1:8080/dash
 * 4. Find "DefenseTuning" in the configuration panel
 * 5. Have a partner push your robot while you adjust sliders
 * 6. Watch telemetry to see position error, velocity, and corrections
 * 7. When tuned, copy the values to NovadConstants.java
 * 
 * CONTROLS:
 * - A: Enable defense
 * - B: Disable defense  
 * - X: Lock position (maximum resistance)
 * - Y: Unlock position
 * - Left stick: Drive (when defense is off or for override testing)
 * - Right stick X: Rotate
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
@Config
@TeleOp(name = "Novad Defense Tuning", group = "Novad")
public class DefenseTuning extends LinearOpMode {

    // ========================================================================================
    // TUNABLE VALUES - Adjust these in panels.bylazar.com!
    // ========================================================================================

    // Velocity PID - Controls immediate push resistance
    public static double VELOCITY_KP = 1.5;
    public static double VELOCITY_KI = 0.2;
    public static double VELOCITY_KD = 0.1;

    // Position PID - Controls return to locked position
    public static double POSITION_KP = 0.8;
    public static double POSITION_KI = 0.15;
    public static double POSITION_KD = 0.05;

    // Heading PID - Controls rotation lock
    public static double HEADING_KP = 2.0;
    public static double HEADING_KI = 0.05;
    public static double HEADING_KD = 0.15;

    // Behavior Settings
    public static double ACTIVATION_DELAY_MS = 75;
    public static double RAMP_TIME_MS = 200;
    public static double JOYSTICK_DEADBAND = 0.08;
    public static double CORRECTION_DEADBAND = 0.25;
    public static double MAX_CORRECTION_POWER = 0.85;

    // Mode toggles
    public static boolean ENABLE_DEFENSE = true;
    public static String DEFENSE_MODE = "NORMAL"; // NORMAL, LOCKED, DISABLED

    // ========================================================================================
    // OpMode Implementation
    // ========================================================================================

    private Novad novad;

    // Your team should replace these with your actual implementations
    private NovadOdometry odometry;
    private NovadDrivetrain drivetrain;

    @Override
    public void runOpMode() throws InterruptedException {
        // Initialize telemetry with FTC Dashboard
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ===== TEAMS: Replace this section with your actual hardware initialization =====
        // Example:
        // odometry = new YourOdometryClass(hardwareMap);
        // drivetrain = new YourDrivetrainClass(hardwareMap);
        
        // For now, we'll show a message about what to do
        telemetry.addLine("=== NOVAD DEFENSE TUNING ===");
        telemetry.addLine("");
        telemetry.addLine("SETUP REQUIRED:");
        telemetry.addLine("Replace the odometry and drivetrain");
        telemetry.addLine("initialization in this file with your");
        telemetry.addLine("actual hardware classes.");
        telemetry.addLine("");
        telemetry.addLine("TUNING INSTRUCTIONS:");
        telemetry.addLine("1. Go to panels.bylazar.com");
        telemetry.addLine("2. Or http://192.168.43.1:8080/dash");
        telemetry.addLine("3. Find 'DefenseTuning' config");
        telemetry.addLine("4. Adjust PID sliders while pushing robot");
        telemetry.addLine("");
        telemetry.addLine("CONTROLS:");
        telemetry.addLine("A = Enable defense");
        telemetry.addLine("B = Disable defense");
        telemetry.addLine("X = Lock position (max resistance)");
        telemetry.addLine("Y = Unlock position");
        telemetry.update();

        // Wait for hardware initialization or skip for testing
        // Comment out the following lines and add your hardware init
        if (odometry == null || drivetrain == null) {
            telemetry.addLine("");
            telemetry.addLine(">>> Hardware not initialized!");
            telemetry.addLine(">>> See comments in DefenseTuning.java");
            telemetry.update();
            
            // Create dummy implementations for testing dashboard connection
            odometry = createDummyOdometry();
            drivetrain = createDummyDrivetrain();
        }

        // Initialize Novad
        novad = new Novad(odometry, drivetrain);

        // Apply initial tuning values
        applyTuningValues();

        telemetry.addLine("");
        telemetry.addLine(">>> Ready! Press START <<<");
        telemetry.update();

        waitForStart();

        while (opModeIsActive()) {
            // Update tuning values from dashboard
            applyTuningValues();

            // Handle gamepad input
            handleControls();

            // Get joystick values
            double leftX = gamepad1.left_stick_x;
            double leftY = -gamepad1.left_stick_y; // Invert Y
            double rightX = gamepad1.right_stick_x;

            // Run defense system
            if (ENABLE_DEFENSE) {
                novad.defense(leftX, leftY, rightX);
            } else {
                drivetrain.drive(leftY, leftX, rightX);
            }

            // Display telemetry
            displayTelemetry(leftX, leftY, rightX);
        }

        // Cleanup
        novad.disable();
    }

    /**
     * Apply tuning values from dashboard to Novad
     */
    private void applyTuningValues() {
        // Update NovadConstants
        NovadConstants.VELOCITY_KP = VELOCITY_KP;
        NovadConstants.VELOCITY_KI = VELOCITY_KI;
        NovadConstants.VELOCITY_KD = VELOCITY_KD;

        NovadConstants.POSITION_KP = POSITION_KP;
        NovadConstants.POSITION_KI = POSITION_KI;
        NovadConstants.POSITION_KD = POSITION_KD;

        NovadConstants.HEADING_KP = HEADING_KP;
        NovadConstants.HEADING_KI = HEADING_KI;
        NovadConstants.HEADING_KD = HEADING_KD;

        NovadConstants.ACTIVATION_DELAY_MS = ACTIVATION_DELAY_MS;
        NovadConstants.RAMP_TIME_MS = RAMP_TIME_MS;
        NovadConstants.JOYSTICK_DEADBAND = JOYSTICK_DEADBAND;
        NovadConstants.CORRECTION_DEADBAND = CORRECTION_DEADBAND;
        NovadConstants.MAX_CORRECTION_POWER = MAX_CORRECTION_POWER;

        // Update controller gains
        if (novad != null) {
            novad.updateGainsFromConstants();
        }
    }

    /**
     * Handle gamepad button presses
     */
    private void handleControls() {
        // A - Enable defense
        if (gamepad1.a) {
            ENABLE_DEFENSE = true;
            DEFENSE_MODE = "NORMAL";
            novad.enable();
        }

        // B - Disable defense
        if (gamepad1.b) {
            ENABLE_DEFENSE = false;
            DEFENSE_MODE = "DISABLED";
            novad.disable();
        }

        // X - Lock position
        if (gamepad1.x) {
            DEFENSE_MODE = "LOCKED";
            novad.lockPosition();
        }

        // Y - Unlock position
        if (gamepad1.y) {
            DEFENSE_MODE = "NORMAL";
            novad.unlockPosition();
        }
    }

    /**
     * Display comprehensive telemetry
     */
    private void displayTelemetry(double leftX, double leftY, double rightX) {
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("      NOVAD DEFENSE TUNING");
        telemetry.addLine("═══════════════════════════════════");
        telemetry.addLine("");

        // Status
        telemetry.addData("Defense Active", ENABLE_DEFENSE ? "✓ YES" : "✗ NO");
        telemetry.addData("Mode", DEFENSE_MODE);
        telemetry.addData("Position Locked", novad.isPositionLocked() ? "✓ LOCKED" : "○ FREE");
        telemetry.addData("Driver Active", novad.isDriverActive() ? "✓ DRIVING" : "○ IDLE");
        telemetry.addLine("");

        // PID Values (formatted nicely)
        telemetry.addLine("─── VELOCITY PID ───");
        telemetry.addData("  P", String.format("%.4f", VELOCITY_KP));
        telemetry.addData("  I", String.format("%.4f", VELOCITY_KI));
        telemetry.addData("  D", String.format("%.4f", VELOCITY_KD));

        telemetry.addLine("─── POSITION PID ───");
        telemetry.addData("  P", String.format("%.4f", POSITION_KP));
        telemetry.addData("  I", String.format("%.4f", POSITION_KI));
        telemetry.addData("  D", String.format("%.4f", POSITION_KD));

        telemetry.addLine("─── HEADING PID ───");
        telemetry.addData("  P", String.format("%.4f", HEADING_KP));
        telemetry.addData("  I", String.format("%.4f", HEADING_KI));
        telemetry.addData("  D", String.format("%.4f", HEADING_KD));
        telemetry.addLine("");

        // Real-time errors and corrections
        telemetry.addLine("─── REAL-TIME DATA ───");
        
        Vector2D posError = novad.getPositionError();
        Vector2D velError = novad.getVelocityError();
        
        telemetry.addData("Position Error", String.format("(%.2f, %.2f) in", posError.x, posError.y));
        telemetry.addData("Position Error Mag", String.format("%.3f in", posError.magnitude()));
        
        telemetry.addData("Velocity Error", String.format("(%.2f, %.2f)", velError.x, velError.y));
        telemetry.addData("Heading Error", String.format("%.2f°", Math.toDegrees(novad.getHeadingError())));

        telemetry.addLine("");
        telemetry.addData("Ramp Multiplier", String.format("%.1f%%", novad.getRampMultiplier() * 100));
        telemetry.addData("Correction Power", String.format("%.3f", novad.getTotalCorrectionMagnitude()));
        telemetry.addLine("");

        // Joystick input
        telemetry.addLine("─── JOYSTICK INPUT ───");
        telemetry.addData("Left X (Strafe)", String.format("%.3f", leftX));
        telemetry.addData("Left Y (Forward)", String.format("%.3f", leftY));
        telemetry.addData("Right X (Rotate)", String.format("%.3f", rightX));
        telemetry.addLine("");

        // Instructions
        telemetry.addLine("─── TUNING TIPS ───");
        telemetry.addLine("• Robot oscillates? Lower kP or raise kD");
        telemetry.addLine("• Too slow to resist? Raise kP");
        telemetry.addLine("• Drifts over time? Raise kI slightly");
        telemetry.addLine("• Jerky start? Increase RAMP_TIME_MS");

        telemetry.update();
    }

    // ========================================================================================
    // Dummy implementations for testing (replace with your real hardware)
    // ========================================================================================

    private NovadOdometry createDummyOdometry() {
        return new NovadOdometry() {
            private double x = 0, y = 0, heading = 0;
            private double vx = 0, vy = 0, omega = 0;

            @Override
            public Vector2D getPosition() { return new Vector2D(x, y); }

            @Override
            public double getX() { return x; }

            @Override
            public double getY() { return y; }

            @Override
            public double getHeading() { return heading; }

            @Override
            public Vector2D getVelocity() { return new Vector2D(vx, vy); }

            @Override
            public double getXVelocity() { return vx; }

            @Override
            public double getYVelocity() { return vy; }

            @Override
            public double getAngularVelocity() { return omega; }

            @Override
            public void update() {
                // In real implementation, this reads encoder values
                // For testing, we simulate some drift
                x += Math.random() * 0.01 - 0.005;
                y += Math.random() * 0.01 - 0.005;
            }

            @Override
            public void setPose(double x, double y, double heading) {
                this.x = x;
                this.y = y;
                this.heading = heading;
            }
        };
    }

    private NovadDrivetrain createDummyDrivetrain() {
        return new NovadDrivetrain() {
            @Override
            public void drive(double forward, double strafe, double rotation) {
                // In real implementation, this sets motor powers
            }

            @Override
            public void driveFieldCentric(double forward, double strafe, double rotation, double heading) {
                drive(forward, strafe, rotation);
            }

            @Override
            public void setMotorPowers(double fl, double fr, double bl, double br) {
                // Set individual motor powers
            }

            @Override
            public void stop() {
                drive(0, 0, 0);
            }

            @Override
            public double getMaxSpeed() { return 1.0; }

            @Override
            public void setMaxSpeed(double maxSpeed) {}
        };
    }
}
