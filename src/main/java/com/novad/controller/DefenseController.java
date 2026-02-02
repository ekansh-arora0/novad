package com.novad.controller;

import com.novad.config.NovadConfig;
import com.novad.interfaces.NovadDrivetrain;
import com.novad.interfaces.NovadOdometry;
import com.novad.util.PIDController;
import com.novad.util.Vector2D;

/**
 * DefenseController - core defense logic using position/velocity/heading PID
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
    // runtime telemetry
    private boolean isDriverActive = false;
    private double lastRampMultiplier = 0.0;
    private Vector2D lastVelocityCorrection = new Vector2D();
    private Vector2D lastPositionCorrection = new Vector2D();

    public DefenseController(NovadOdometry odometry, NovadDrivetrain drivetrain, NovadConfig config) {
        this.odometry = odometry;
        this.drivetrain = drivetrain;
        this.config = config;

        // Simple PID controllers - tuned later via tuners
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
        // For now tuners set PID controllers directly; placeholder
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

    public void enable() {
        this.enabled = true;
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

        // Driver active check
        boolean driverActive = Math.abs(driverX) > config.getDriverOverrideThreshold()
            || Math.abs(driverY) > config.getDriverOverrideThreshold()
            || Math.abs(driverRotation) > config.getDriverOverrideThreshold();

        this.isDriverActive = driverActive;

        if (driverActive) {
            // Pass through driver commands (driver controls full motion)
            drivetrain.drive(driverY, driverX, driverRotation);
            return;
        }

        // If position not locked, maintain last locked position
        if (!positionLocked) {
            // set a soft lock at current pose (hold position)
            lockPosition();
        }

        // Compute errors
        Vector2D currentPos = odometry.getPosition();
        double currentHeading = odometry.getHeading();

        Vector2D posError = lockedPosition.subtract(currentPos);
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
        double corrX = posXOut + velXOut;
        double corrY = posYOut + velYOut;

        // Apply ramp multiplier based on how long defense has been active
        double ramp = 1.0;
        if (activationStartTime > 0) {
            double elapsed = (System.currentTimeMillis() - activationStartTime) / 1000.0;
            ramp = Math.min(1.0, elapsed / config.getRampUpTime());
        }

        corrX *= ramp;
        corrY *= ramp;
        headingOut *= ramp;

        // Save telemetry
        lastRampMultiplier = ramp;
        lastVelocityCorrection.set(velXOut, velYOut);
        lastPositionCorrection.set(posXOut, posYOut);

        // Send corrections to drivetrain (note: drivetrain.drive expects forward, strafe, rotation)
        drivetrain.drive(corrY, corrX, headingOut);
    }

    // Telemetry getters
    public Vector2D getPositionError() { return lockedPosition; }
    public Vector2D getVelocityError() { return odometry.getVelocity(); }
    public double getHeadingError() { return lockedHeading - odometry.getHeading(); }
    public Vector2D getLockedPosition() { return new Vector2D(lockedPosition); }
    public double getLockedHeading() { return lockedHeading; }

    public boolean isDriverActive() { return isDriverActive; }
    public double getRampMultiplier() { return lastRampMultiplier; }
    public double getTotalCorrectionMagnitude() {
        return lastVelocityCorrection.add(lastPositionCorrection).magnitude();
    }

    public Vector2D getVelocityCorrection() { return new Vector2D(lastVelocityCorrection); }
    public Vector2D getPositionCorrection() { return new Vector2D(lastPositionCorrection); }
}
