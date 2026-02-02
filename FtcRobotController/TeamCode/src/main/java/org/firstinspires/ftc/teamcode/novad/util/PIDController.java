package org.firstinspires.ftc.teamcode.novad.util;

/**
 * PID Controller implementation for Novad defense system
 * Includes anti-windup, derivative filtering, and reset functionality
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class PIDController {

    // PID gains
    private double kP;
    private double kI;
    private double kD;

    // State variables
    private double integral;
    private double previousError;
    private long previousTimeNanos;
    private boolean firstUpdate;

    // Configuration
    private double maxIntegral;
    private double integralZone;
    private boolean enableDerivativeFilter;
    private double derivativeFilterAlpha;
    private double filteredDerivative;

    // Output limits
    private double minOutput;
    private double maxOutput;

    /**
     * Create a PID controller with specified gains
     * @param kP Proportional gain
     * @param kI Integral gain
     * @param kD Derivative gain
     */
    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;

        this.integral = 0;
        this.previousError = 0;
        this.previousTimeNanos = 0;
        this.firstUpdate = true;

        this.maxIntegral = 1.0;
        this.integralZone = Double.MAX_VALUE;
        this.enableDerivativeFilter = true;
        this.derivativeFilterAlpha = 0.8;
        this.filteredDerivative = 0;

        this.minOutput = -1.0;
        this.maxOutput = 1.0;
    }

    /**
     * Create a PID controller with default gains (all zeros)
     */
    public PIDController() {
        this(0, 0, 0);
    }

    /**
     * Calculate PID output for current error
     * @param error Current error (setpoint - measurement)
     * @return PID correction value
     */
    public double calculate(double error) {
        long currentTimeNanos = System.nanoTime();

        // Handle first update
        if (firstUpdate) {
            previousTimeNanos = currentTimeNanos;
            previousError = error;
            firstUpdate = false;
            return kP * error; // Only P term on first update
        }

        // Calculate delta time in seconds
        double dt = (currentTimeNanos - previousTimeNanos) / 1_000_000_000.0;
        previousTimeNanos = currentTimeNanos;

        // Prevent division by zero
        if (dt <= 0) {
            dt = 0.02; // Default to 20ms
        }

        // Proportional term
        double pTerm = kP * error;

        // Integral term with anti-windup
        if (Math.abs(error) < integralZone) {
            integral += error * dt;
            // Clamp integral to prevent windup
            integral = Math.max(-maxIntegral, Math.min(maxIntegral, integral));
        } else {
            // Reset integral if error is outside integral zone
            integral = 0;
        }
        double iTerm = kI * integral;

        // Derivative term (on error)
        double rawDerivative = (error - previousError) / dt;
        
        // Optional low-pass filter on derivative to reduce noise
        if (enableDerivativeFilter) {
            filteredDerivative = derivativeFilterAlpha * filteredDerivative + 
                                 (1 - derivativeFilterAlpha) * rawDerivative;
        } else {
            filteredDerivative = rawDerivative;
        }
        double dTerm = kD * filteredDerivative;

        // Update previous error
        previousError = error;

        // Calculate total output
        double output = pTerm + iTerm + dTerm;

        // Clamp output
        output = Math.max(minOutput, Math.min(maxOutput, output));

        return output;
    }

    /**
     * Calculate PID output for error between setpoint and measurement
     * @param setpoint Desired value
     * @param measurement Current value
     * @return PID correction value
     */
    public double calculate(double setpoint, double measurement) {
        return calculate(setpoint - measurement);
    }

    /**
     * Reset the controller state
     * Call this when starting a new control sequence
     */
    public void reset() {
        integral = 0;
        previousError = 0;
        filteredDerivative = 0;
        firstUpdate = true;
        previousTimeNanos = System.nanoTime();
    }

    /**
     * Update PID gains
     * @param kP New proportional gain
     * @param kI New integral gain
     * @param kD New derivative gain
     */
    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Set proportional gain only
     * @param kP New proportional gain
     */
    public void setP(double kP) {
        this.kP = kP;
    }

    /**
     * Set integral gain only
     * @param kI New integral gain
     */
    public void setI(double kI) {
        this.kI = kI;
    }

    /**
     * Set derivative gain only
     * @param kD New derivative gain
     */
    public void setD(double kD) {
        this.kD = kD;
    }

    /**
     * Get current proportional gain
     * @return kP value
     */
    public double getP() {
        return kP;
    }

    /**
     * Get current integral gain
     * @return kI value
     */
    public double getI() {
        return kI;
    }

    /**
     * Get current derivative gain
     * @return kD value
     */
    public double getD() {
        return kD;
    }

    /**
     * Set maximum integral accumulation (anti-windup)
     * @param maxIntegral Maximum integral value
     */
    public void setMaxIntegral(double maxIntegral) {
        this.maxIntegral = Math.abs(maxIntegral);
    }

    /**
     * Set integral zone - integral only accumulates when error is within this zone
     * @param zone Error threshold for integral accumulation
     */
    public void setIntegralZone(double zone) {
        this.integralZone = Math.abs(zone);
    }

    /**
     * Set output limits
     * @param min Minimum output value
     * @param max Maximum output value
     */
    public void setOutputLimits(double min, double max) {
        this.minOutput = min;
        this.maxOutput = max;
    }

    /**
     * Enable or disable derivative filtering
     * @param enable True to enable low-pass filter on derivative
     */
    public void setDerivativeFilter(boolean enable) {
        this.enableDerivativeFilter = enable;
    }

    /**
     * Set derivative filter alpha (smoothing factor)
     * Higher values = more smoothing, slower response
     * @param alpha Filter coefficient (0.0 to 1.0)
     */
    public void setDerivativeFilterAlpha(double alpha) {
        this.derivativeFilterAlpha = Math.max(0, Math.min(1, alpha));
    }

    /**
     * Get the current integral value
     * @return Accumulated integral
     */
    public double getIntegral() {
        return integral;
    }

    /**
     * Get the current error (from last calculation)
     * @return Previous error value
     */
    public double getError() {
        return previousError;
    }

    /**
     * Check if controller has been reset and not yet updated
     * @return True if waiting for first update
     */
    public boolean isFirstUpdate() {
        return firstUpdate;
    }

    @Override
    public String toString() {
        return String.format("PID(P=%.3f, I=%.3f, D=%.3f)", kP, kI, kD);
    }
}
