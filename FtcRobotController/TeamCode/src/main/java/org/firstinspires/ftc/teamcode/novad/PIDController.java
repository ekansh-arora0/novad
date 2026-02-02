package org.firstinspires.ftc.teamcode.novad;

/**
 * Simple PID Controller
 * 
 * Used internally by Novad for position and heading correction.
 */
public class PIDController {
    private double kP, kI, kD;
    
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;
    private boolean firstRun = true;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Update PID gains (for live tuning via FTC Dashboard)
     */
    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Calculate PID output
     * 
     * @param error Current error (target - actual)
     * @return Correction value
     */
    public double calculate(double error) {
        long currentTime = System.nanoTime();
        
        if (firstRun) {
            lastTime = currentTime;
            lastError = error;
            firstRun = false;
            return kP * error;
        }

        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        if (dt <= 0) dt = 0.001; // Prevent division by zero

        // Proportional term
        double p = kP * error;

        // Integral term (with anti-windup)
        integral += error * dt;
        integral = clamp(integral, -1.0 / (kI + 0.001), 1.0 / (kI + 0.001)); // Anti-windup
        double i = kI * integral;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double d = kD * derivative;

        // Update state
        lastError = error;
        lastTime = currentTime;

        return p + i + d;
    }

    /**
     * Reset the controller state
     * Call this when re-enabling or switching targets
     */
    public void reset() {
        integral = 0;
        lastError = 0;
        firstRun = true;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
