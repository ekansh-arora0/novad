package org.firstinspires.ftc.teamcode.novad;

/**
 * PIDF Controller with filtered derivative (matches Pedro Pathing style)
 * 
 * P = Proportional: Main correction force, proportional to error
 * I = Integral: Accumulates error over time, fixes steady-state drift
 * D = Derivative: Dampens oscillation, filtered to reduce noise
 * F = Feedforward: Scales with error to help with steady-state
 */
public class PIDController {
    private double kP, kI, kD, kF;
    
    private double integral = 0;
    private double lastError = 0;
    private double filteredDerivative = 0;  // Low-pass filtered derivative
    private long lastTime = 0;
    private boolean firstRun = true;
    
    // Low-pass filter coefficient for derivative (0 = no filter, 1 = full filter)
    // Higher = smoother but slower response
    private static final double DERIVATIVE_FILTER = 0.8;

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
     * Update PIDF gains including feedforward (alias for setGains)
     */
    public void setPIDF(double kP, double kI, double kD, double kF) {
        setGains(kP, kI, kD, kF);
    }

    /**
     * Calculate PIDF output - matches Pedro Pathing formula
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
            // First run: just P term, no derivative or integral yet
            return kP * error;
        }

        double dt = (currentTime - lastTime) / 1e9; // Convert to seconds
        if (dt <= 0) dt = 0.001; // Prevent division by zero
        if (dt > 0.1) dt = 0.1; // Cap to prevent huge jumps

        // Proportional term
        double p = kP * error;

        // Integral term (with anti-windup)
        integral += error * dt;
        double maxIntegral = (kI > 0.0001) ? (1.0 / kI) : 10.0;
        integral = clamp(integral, -maxIntegral, maxIntegral);
        double i = kI * integral;

        // Derivative term with low-pass filter (reduces noise-induced jitter)
        // This matches Pedro Pathing's FilteredPIDFController approach
        double rawDerivative = (error - lastError) / dt;
        filteredDerivative = DERIVATIVE_FILTER * filteredDerivative + (1 - DERIVATIVE_FILTER) * rawDerivative;
        double d = kD * filteredDerivative;

        // Feedforward term - proportional to error (Pedro style)
        // This scales smoothly rather than jumping with signum
        double f = kF * error;

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
        filteredDerivative = 0;
        firstRun = true;
    }

    private double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
