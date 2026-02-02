package org.firstinspires.ftc.teamcode.novad.controller;

import org.firstinspires.ftc.teamcode.novad.config.NovadConfig;
import org.firstinspires.ftc.teamcode.novad.interfaces.NovadDrivetrain;
import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;
import org.firstinspires.ftc.teamcode.novad.util.PIDController;
import org.firstinspires.ftc.teamcode.novad.util.Vector2D;

/**
 * DefenseController - Core defense logic with PREDICTIVE DEFENSE
 * 
 * ═══════════════════════════════════════════════════════════════════════════════
 * PREDICTIVE DEFENSE: What makes Novad different
 * ═══════════════════════════════════════════════════════════════════════════════
 * 
 * Traditional defense libraries react AFTER you've been pushed.
 * Novad predicts WHERE you'll be pushed and counters BEFORE you get there.
 * 
 * How it works:
 * 1. Monitors acceleration (sudden change in velocity = push detected)
 * 2. Predicts position X milliseconds in the future
 * 3. Applies counter-force immediately, not after displacement
 * 4. Result: Faster response without sacrificing accuracy
 * 
 * The prediction lookahead is configurable (PREDICTION_LOOKAHEAD_MS).
 * Higher values = faster response but potentially less stable.
 * Lower values = more stable but slower response.
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class DefenseController {

    private final NovadOdometry odometry;
    private final NovadDrivetrain drivetrain;
    private NovadConfig config;

    // PID controllers
    private final PIDController velocityXPID;
    private final PIDController velocityYPID;
    private final PIDController positionXPID;
    private final PIDController positionYPID;
    private final PIDController headingPID;

    // State
    private boolean enabled = false;
    private boolean positionLocked = false;
    private Vector2D lockedPosition = new Vector2D();
    private double lockedHeading = 0.0;
    private long activationStartTime = 0;
    
    // Predictive defense state
    private boolean predictiveEnabled = true;
    private double predictionLookaheadMs = 50;
    private double accelTriggerThreshold = 20.0; // in/s²
    private double instantBoostMultiplier = 1.5;
    
    // Velocity/acceleration tracking
    private Vector2D lastVelocity = new Vector2D();
    private Vector2D acceleration = new Vector2D();
    private long lastUpdateTime = 0;
    
    // Runtime telemetry
    private boolean isDriverActive = false;
    private double lastRampMultiplier = 0.0;
    private Vector2D lastVelocityCorrection = new Vector2D();
    private Vector2D lastPositionCorrection = new Vector2D();
    private boolean isPredictiveActive = false;
    private double currentBoostMultiplier = 1.0;

    public DefenseController(NovadOdometry odometry, NovadDrivetrain drivetrain, NovadConfig config) {
        this.odometry = odometry;
        this.drivetrain = drivetrain;
        this.config = config;

        // Initialize PID controllers with config values
        this.velocityXPID = new PIDController(0.02, 0.0, 0.001);
        this.velocityYPID = new PIDController(0.02, 0.0, 0.001);
        this.positionXPID = new PIDController(0.05, 0.0, 0.005);
        this.positionYPID = new PIDController(0.05, 0.0, 0.005);
        this.headingPID = new PIDController(0.01, 0.0, 0.001);

        // Configure from config
        configureFromConfig();
    }

    private void configureFromConfig() {
        double maxOutput = config.getMaxCorrectionPower();
        double maxIntegral = config.getMaxIntegral();

        velocityXPID.setOutputLimits(-maxOutput, maxOutput);
        velocityYPID.setOutputLimits(-maxOutput, maxOutput);
        positionXPID.setOutputLimits(-maxOutput, maxOutput);
        positionYPID.setOutputLimits(-maxOutput, maxOutput);
        headingPID.setOutputLimits(-maxOutput, maxOutput);

        velocityXPID.setMaxIntegral(maxIntegral);
        velocityYPID.setMaxIntegral(maxIntegral);
        positionXPID.setMaxIntegral(maxIntegral);
        positionYPID.setMaxIntegral(maxIntegral);
        headingPID.setMaxIntegral(maxIntegral);
    }

    public void updateGainsFromConfig(NovadConfig cfg) {
        this.config = cfg;
        configureFromConfig();
    }

    public void setVelocityGains(double kP, double kI, double kD) {
        velocityXPID.setGains(kP, kI, kD);
        velocityYPID.setGains(kP, kI, kD);
    }

    public void setPositionGains(double kP, double kI, double kD) {
        positionXPID.setGains(kP, kI, kD);
        positionYPID.setGains(kP, kI, kD);
    }

    public void setHeadingGains(double kP, double kI, double kD) {
        headingPID.setGains(kP, kI, kD);
    }
    
    // ═══════════════════════════════════════════════════════════════════════════
    // PREDICTIVE DEFENSE CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════════
    
    /**
     * Enable/disable predictive defense
     */
    public void setPredictiveEnabled(boolean enabled) {
        this.predictiveEnabled = enabled;
    }
    
    /**
     * Set how far ahead to predict (in milliseconds)
     * Higher = faster response, potentially less stable
     * Lower = more stable, slower response
     * Recommended: 30-80ms
     */
    public void setPredictionLookahead(double milliseconds) {
        this.predictionLookaheadMs = milliseconds;
    }
    
    /**
     * Set acceleration threshold that triggers instant response
     * When acceleration exceeds this, boost multiplier is applied
     */
    public void setAccelTriggerThreshold(double inchesPerSecondSquared) {
        this.accelTriggerThreshold = inchesPerSecondSquared;
    }
    
    /**
     * Set the boost multiplier when sudden acceleration is detected
     * 1.0 = no boost, 2.0 = double response
     */
    public void setInstantBoostMultiplier(double multiplier) {
        this.instantBoostMultiplier = multiplier;
    }

    public void enable() {
        this.enabled = true;
        this.lastUpdateTime = System.nanoTime();
    }

    public void disable() {
        this.enabled = false;
        drivetrain.stop();
    }

    public boolean isEnabled() { return enabled; }
    public boolean isPositionLocked() { return positionLocked; }

    public void lockPosition() {
        this.positionLocked = true;
        this.lockedPosition = odometry.getPosition();
        this.lockedHeading = odometry.getHeading();
        this.activationStartTime = System.currentTimeMillis();
    }

    public void unlockPosition() {
        this.positionLocked = false;
    }

    public void togglePositionLock() {
        if (positionLocked) unlockPosition(); else lockPosition();
    }

    public void update(double driverX, double driverY, double driverRotation) {
        // Update odometry first
        odometry.update();
        
        // Calculate acceleration
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastUpdateTime) / 1_000_000_000.0;
        
        Vector2D currentVelocity = odometry.getVelocity();
        if (dt > 0.001) {
            acceleration = new Vector2D(
                (currentVelocity.x - lastVelocity.x) / dt,
                (currentVelocity.y - lastVelocity.y) / dt
            );
            lastVelocity = currentVelocity;
        }
        lastUpdateTime = currentTime;

        // Driver active check
        boolean driverActive = Math.abs(driverX) > config.getDriverOverrideThreshold()
            || Math.abs(driverY) > config.getDriverOverrideThreshold()
            || Math.abs(driverRotation) > config.getDriverOverrideThreshold();

        this.isDriverActive = driverActive;

        if (driverActive) {
            // Pass through driver commands
            drivetrain.drive(driverY, driverX, driverRotation);
            return;
        }

        // If position not locked, maintain last locked position
        if (!positionLocked) {
            lockPosition();
        }

        // ═══════════════════════════════════════════════════════════════════════
        // PREDICTIVE DEFENSE CALCULATION
        // ═══════════════════════════════════════════════════════════════════════
        
        Vector2D currentPos = odometry.getPosition();
        double currentHeading = odometry.getHeading();
        
        // Calculate predicted position if predictive defense is enabled
        Vector2D targetPos = lockedPosition;
        if (predictiveEnabled) {
            double lookaheadSeconds = predictionLookaheadMs / 1000.0;
            
            // Predict where robot will be based on current velocity and acceleration
            Vector2D predictedDisplacement = new Vector2D(
                currentVelocity.x * lookaheadSeconds + 0.5 * acceleration.x * lookaheadSeconds * lookaheadSeconds,
                currentVelocity.y * lookaheadSeconds + 0.5 * acceleration.y * lookaheadSeconds * lookaheadSeconds
            );
            
            // Use predicted position for error calculation
            Vector2D predictedPos = currentPos.add(predictedDisplacement);
            
            // Calculate error from predicted position to locked position
            // This makes us counter the push BEFORE we actually get pushed there
            targetPos = lockedPosition;
            currentPos = predictedPos;
        }
        
        // Check for sudden acceleration (impact detection)
        double accelMagnitude = acceleration.magnitude();
        isPredictiveActive = predictiveEnabled && accelMagnitude > accelTriggerThreshold;
        
        // Apply boost multiplier if sudden acceleration detected
        currentBoostMultiplier = isPredictiveActive ? instantBoostMultiplier : 1.0;

        // Compute errors
        Vector2D posError = targetPos.subtract(currentPos);
        Vector2D vel = odometry.getVelocity();
        Vector2D velError = new Vector2D(-vel.x, -vel.y);
        double headingError = lockedHeading - currentHeading;

        // PID outputs
        double posXOut = positionXPID.calculate(posError.x);
        double posYOut = positionYPID.calculate(posError.y);
        double velXOut = velocityXPID.calculate(velError.x);
        double velYOut = velocityYPID.calculate(velError.y);
        double headingOut = headingPID.calculate(headingError);

        // Combine corrections (position + velocity)
        double corrX = (posXOut + velXOut) * currentBoostMultiplier;
        double corrY = (posYOut + velYOut) * currentBoostMultiplier;
        headingOut *= currentBoostMultiplier;

        // Apply ramp multiplier based on how long defense has been active
        double ramp = 1.0;
        if (activationStartTime > 0) {
            double elapsed = (System.currentTimeMillis() - activationStartTime) / 1000.0;
            ramp = Math.min(1.0, elapsed / config.getRampUpTime());
        }

        corrX *= ramp;
        corrY *= ramp;
        headingOut *= ramp;

        // Clamp to max correction power
        double maxPower = config.getMaxCorrectionPower();
        corrX = Math.max(-maxPower, Math.min(maxPower, corrX));
        corrY = Math.max(-maxPower, Math.min(maxPower, corrY));
        headingOut = Math.max(-maxPower, Math.min(maxPower, headingOut));

        // Save telemetry
        lastRampMultiplier = ramp;
        lastVelocityCorrection = new Vector2D(velXOut, velYOut);
        lastPositionCorrection = new Vector2D(posXOut, posYOut);

        // Send corrections to drivetrain
        drivetrain.drive(corrY, corrX, headingOut);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // TELEMETRY GETTERS
    // ═══════════════════════════════════════════════════════════════════════════
    
    public Vector2D getPositionError() { 
        return lockedPosition.subtract(odometry.getPosition()); 
    }
    
    public Vector2D getVelocityError() { 
        return odometry.getVelocity(); 
    }
    
    public double getHeadingError() { 
        return lockedHeading - odometry.getHeading(); 
    }
    
    public Vector2D getLockedPosition() { 
        return new Vector2D(lockedPosition); 
    }
    
    public double getLockedHeading() { 
        return lockedHeading; 
    }

    public boolean isDriverActive() { return isDriverActive; }
    public double getRampMultiplier() { return lastRampMultiplier; }
    
    public double getTotalCorrectionMagnitude() {
        return lastVelocityCorrection.add(lastPositionCorrection).magnitude();
    }

    public Vector2D getVelocityCorrection() { return new Vector2D(lastVelocityCorrection); }
    public Vector2D getPositionCorrection() { return new Vector2D(lastPositionCorrection); }
    
    // Predictive defense telemetry
    public boolean isPredictiveActive() { return isPredictiveActive; }
    public double getCurrentBoostMultiplier() { return currentBoostMultiplier; }
    public Vector2D getAcceleration() { return new Vector2D(acceleration); }
    public double getAccelerationMagnitude() { return acceleration.magnitude(); }
}
