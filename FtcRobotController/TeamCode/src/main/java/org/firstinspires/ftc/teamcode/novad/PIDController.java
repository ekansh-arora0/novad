package org.firstinspires.ftc.teamcode.novad;

/**
 * PIDF Controller
 * 
 * P = Proportional: Main correction force, proportional to error
 * I = Integral: Accumulates error over time, fixes steady-state drift
 * D = Derivative: Dampens oscillation by reacting to rate of change
 * F = Feedforward: Constant offset to overcome static friction (motor jitter)
 */
public class PIDController {
    private double kP, kI, kD, kF;
    
    private double integral = 0;
    private double lastError = 0;
    private long lastTime = 0;
    private boolean firstRun = true;

    public PIDController(double kP, double kI, double kD) {
        this(kP, kI, kD, 0);
    }

    public PIDController(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Update PIDF gains (for live tuning via FTC Dashboard)
     */
    public void setGains(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    /**
     * Update PIDF gains including feedforward
     */
    public void setGains(double kP, double kI, double kD, double kF) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.kF = kF;
    }

    /**
     * Calculate PIDF output
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
            return kP * error + Math.signum(error) * kF;
        }

        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        if (dt <= 0) dt = 0.001; // Prevent division by zero

        // Proportional term
        double p = kP * error;

        // Integral term (with anti-windup)
        integral += error * dt;
        double maxIntegral = (kI > 0.0001) ? (1.0 / kI) : 10.0;
        integral = clamp(integral, -maxIntegral, maxIntegral);
        double i = kI * integral;

        // Derivative term
        double derivative = (error - lastError) / dt;
        double d = kD * derivative;

        // Feedforward term (overcomes static friction)
        // Applied in the direction of error, helps with motor jitter
        double f = Math.signum(error) * kF;

        // Update state
        lastError = error;
        lastTime = currentTime;

        return p + i + d + f;
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
