package org.firstinspires.ftc.teamcode.novad;

import org.firstinspires.ftc.teamcode.novad.config.NovadConfig;
import org.firstinspires.ftc.teamcode.novad.controller.DefenseController;
import org.firstinspires.ftc.teamcode.novad.interfaces.NovadDrivetrain;
import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;
import org.firstinspires.ftc.teamcode.novad.util.Vector2D;

/**
 * Novad - FTC Defense Library
 * 
 * Stop getting pushed around. Add push resistance to your TeleOp with a single method call.
 * 
 * QUICK START:
 * <pre>
 * // Initialize (once in init)
 * Novad novad = new Novad(odometry, drivetrain);
 * 
 * // Use in TeleOp loop
 * novad.defense(gamepad1.left_stick_x, gamepad1.left_stick_y, gamepad1.right_stick_x);
 * </pre>
 * 
 * FEATURES:
 * - Dual-layer PID correction (velocity + position)
 * - Automatic driver override detection
 * - Gradual ramp-up to prevent jerky movements
 * - Position lock mode for maximum resistance
 * - Works with any odometry system
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 * @see <a href="https://novad.dev">Documentation</a>
 */
public class Novad {

    /** Library version */
    public static final String VERSION = "1.0.0";

    // Core components
    private final DefenseController controller;
    private final NovadOdometry odometry;
    private final NovadDrivetrain drivetrain;

    // State
    private boolean enabled;

    /**
     * Create a new Novad defense system
     * 
     * @param odometry Your odometry implementation (NovadOdometry interface)
     * @param drivetrain Your drivetrain implementation (NovadDrivetrain interface)
     */
    public Novad(NovadOdometry odometry, NovadDrivetrain drivetrain) {
        this.odometry = odometry;
        this.drivetrain = drivetrain;
        this.controller = new DefenseController(odometry, drivetrain, NovadConfig.getDefault());
        this.enabled = false;
    }

    /**
     * Create Novad with a specific configuration
     */
    public Novad(NovadOdometry odometry, NovadDrivetrain drivetrain, NovadConfig config) {
        this.odometry = odometry;
        this.drivetrain = drivetrain;
        this.controller = new DefenseController(odometry, drivetrain, config);
        this.enabled = false;
    }

    // ========================================================================================
    // MAIN API METHODS - These are the methods teams will use
    // ========================================================================================

    /**
     * Enable defense and apply corrections while passing through driver input
     * 
     * Call this method every loop iteration in your TeleOp. It will:
     * - Pass through your joystick inputs to the drivetrain
     * - Detect when the robot is being pushed (movement without joystick input)
     * - Apply counter-force to resist the push
     * - Automatically disable corrections when you move the joysticks
     * 
     * Example:
     * <pre>
     * while (opModeIsActive()) {
     *     novad.defense(
     *         gamepad1.left_stick_x,   // Strafe
     *         -gamepad1.left_stick_y,  // Forward (note: invert Y!)
     *         gamepad1.right_stick_x   // Rotate
     *     );
     * }
     * </pre>
     * 
     * @param leftStickX Strafe input (-1 to 1, positive = right)
     * @param leftStickY Forward input (-1 to 1, positive = forward)
     * @param rightStickX Rotation input (-1 to 1, positive = counter-clockwise)
     */
    public void defense(double leftStickX, double leftStickY, double rightStickX) {
        if (!enabled) {
            enable();
        }
        controller.update(leftStickX, leftStickY, rightStickX);
    }

    /**
     * Lock the robot's current position for maximum push resistance
     * 
     * When locked, the robot will use all available motor power to resist
     * any displacement and return to the locked position. The driver can
     * still override by moving the joysticks.
     * 
     * Example:
     * <pre>
     * if (gamepad1.a) {
     *     novad.lockPosition();
     * }
     * </pre>
     */
    public void lockPosition() {
        if (!enabled) {
            enable();
        }
        controller.lockPosition();
    }

    /**
     * Release the position lock
     * 
     * The robot will still resist pushes, but won't try to return
     * to a specific position. Call this when the driver wants to
     * move freely again.
     * 
     * Example:
     * <pre>
     * if (gamepad1.b) {
     *     novad.unlockPosition();
     * }
     * </pre>
     */
    public void unlockPosition() {
        controller.unlockPosition();
    }

    /**
     * Toggle position lock on/off
     * 
     * Convenient for using with a single button press.
     * 
     * Example:
     * <pre>
     * if (gamepad1.a && !lastA) {  // Rising edge
     *     novad.togglePositionLock();
     * }
     * lastA = gamepad1.a;
     * </pre>
     */
    public void togglePositionLock() {
        controller.togglePositionLock();
    }

    /**
     * Enable the defense system
     * 
     * Called automatically by defense(), but you can call it
     * explicitly if needed.
     */
    public void enable() {
        enabled = true;
        controller.enable();
    }

    /**
     * Disable the defense system completely
     * 
     * Stops all motors and turns off all corrections.
     * Call this at the end of the match or when you don't need defense.
     */
    public void disable() {
        enabled = false;
        controller.disable();
    }

    // ========================================================================================
    // CONFIGURATION METHODS
    // ========================================================================================

    /**
     * Update PID gains from a NovadConfig
     * 
     * Call this after changing config or during live tuning.
     */
    public void updateGainsFromConfig(NovadConfig config) {
        controller.updateGainsFromConfig(config);
    }

    /**
     * Set heading PID gains (convenience alias)
     */
    public void setHeadingGains(double kP, double kI, double kD) {
        controller.setHeadingGains(kP, kI, kD);
    }

    /**
     * Set velocity PID gains
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setVelocityPID(double kP, double kI, double kD) {
        controller.setVelocityGains(kP, kI, kD);
    }

    /**
     * Set position PID gains
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setPositionPID(double kP, double kI, double kD) {
        controller.setPositionGains(kP, kI, kD);
    }

    /**
     * Set heading PID gains
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public void setHeadingPID(double kP, double kI, double kD) {
        controller.setHeadingGains(kP, kI, kD);
    }

    // ========================================================================================
    // STATUS METHODS - For telemetry and debugging
    // ========================================================================================

    /**
     * Check if the defense system is enabled
     * @return True if enabled
     */
    public boolean isEnabled() {
        return enabled && controller.isEnabled();
    }

    /**
     * Check if position is locked
     * @return True if position locked
     */
    public boolean isPositionLocked() {
        return controller.isPositionLocked();
    }

    /**
     * Check if driver is currently providing input
     * @return True if joysticks exceed deadband
     */
    public boolean isDriverActive() {
        return controller.isDriverActive();
    }

    /**
     * Get the current position error (locked - current)
     * @return Position error vector in inches
     */
    public Vector2D getPositionError() {
        return controller.getPositionError();
    }

    /**
     * Get the current velocity error (intended - actual)
     * @return Velocity error vector
     */
    public Vector2D getVelocityError() {
        return controller.getVelocityError();
    }

    /**
     * Get the current heading error
     * @return Heading error in radians
     */
    public double getHeadingError() {
        return controller.getHeadingError();
    }

    /**
     * Get the current ramp multiplier (0.0 to 1.0)
     * @return Ramp multiplier
     */
    public double getRampMultiplier() {
        return controller.getRampMultiplier();
    }

    /**
     * Get the total correction magnitude being applied
     * @return Correction magnitude
     */
    public double getTotalCorrectionMagnitude() {
        return controller.getTotalCorrectionMagnitude();
    }

    /**
     * Get the locked position
     * @return Locked position vector
     */
    public Vector2D getLockedPosition() {
        return controller.getLockedPosition();
    }

    /**
     * Get the locked heading
     * @return Locked heading in radians
     */
    public double getLockedHeading() {
        return controller.getLockedHeading();
    }

    // ========================================================================================
    // PREDICTIVE DEFENSE METHODS
    // ========================================================================================

    /**
     * Enable/disable predictive defense
     * When enabled, Novad predicts where you'll be pushed and counters BEFORE you get there.
     */
    public void setPredictiveEnabled(boolean enabled) {
        controller.setPredictiveEnabled(enabled);
    }

    /**
     * Set how far ahead to predict (in milliseconds)
     * Higher = faster response, potentially less stable
     * Lower = more stable, slower response
     * Recommended: 30-80ms
     */
    public void setPredictionLookahead(double milliseconds) {
        controller.setPredictionLookahead(milliseconds);
    }

    /**
     * Set acceleration threshold that triggers instant response
     * When acceleration exceeds this, boost multiplier is applied
     */
    public void setAccelTriggerThreshold(double inchesPerSecondSquared) {
        controller.setAccelTriggerThreshold(inchesPerSecondSquared);
    }

    /**
     * Set the boost multiplier when sudden acceleration is detected
     * 1.0 = no boost, 2.0 = double response
     */
    public void setInstantBoostMultiplier(double multiplier) {
        controller.setInstantBoostMultiplier(multiplier);
    }

    /**
     * Check if predictive defense is currently active (impact detected)
     */
    public boolean isPredictiveActive() {
        return controller.isPredictiveActive();
    }

    /**
     * Get the current boost multiplier being applied
     */
    public double getCurrentBoostMultiplier() {
        return controller.getCurrentBoostMultiplier();
    }

    /**
     * Get the current acceleration magnitude
     */
    public double getAccelerationMagnitude() {
        return controller.getAccelerationMagnitude();
    }

    // ========================================================================================
    // UTILITY METHODS
    // ========================================================================================

    /**
     * Get the library version
     * @return Version string
     */
    public static String getVersion() {
        return VERSION;
    }

    /**
     * Get access to the underlying odometry system
     * @return Odometry instance
     */
    public NovadOdometry getOdometry() {
        return odometry;
    }

    /**
     * Get access to the underlying drivetrain
     * @return Drivetrain instance
     */
    public NovadDrivetrain getDrivetrain() {
        return drivetrain;
    }

    /**
     * Get access to the defense controller for advanced usage
     * @return DefenseController instance
     */
    public DefenseController getController() {
        return controller;
    }
}
