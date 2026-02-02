package com.novad.config;

/**
 * NovadConfig - Complete robot configuration using builder pattern
 * 
 * EXAMPLE USAGE:
 * <pre>
 * NovadConfig config = new NovadConfig.Builder()
 *     // Motor names (must match your robot configuration)
 *     .leftFrontMotorName("left_front_drive")
 *     .leftRearMotorName("left_back_drive")
 *     .rightFrontMotorName("right_front_drive")
 *     .rightRearMotorName("right_back_drive")
 *     
 *     // Motor directions
 *     .leftFrontMotorDirection(MotorDirection.REVERSE)
 *     .leftRearMotorDirection(MotorDirection.REVERSE)
 *     .rightFrontMotorDirection(MotorDirection.FORWARD)
 *     .rightRearMotorDirection(MotorDirection.FORWARD)
 *     
 *     // PIDF tuning coefficients
 *     .translationalPIDFCoefficients(new PIDFCoefficients(0.046, 0, 0.01, 0.02))
 *     .headingPIDFCoefficients(new PIDFCoefficients(0.67, 0, 0.006, 0.02))
 *     
 *     .build();
 * </pre>
 */
public class NovadConfig {

    // ═══════════════════════════════════════════════════════════════════════════
    // MOTOR CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════════
    
    private final String leftFrontMotorName;
    private final String leftRearMotorName;
    private final String rightFrontMotorName;
    private final String rightRearMotorName;
    
    private final MotorDirection leftFrontMotorDirection;
    private final MotorDirection leftRearMotorDirection;
    private final MotorDirection rightFrontMotorDirection;
    private final MotorDirection rightRearMotorDirection;

    // ═══════════════════════════════════════════════════════════════════════════
    // PIDF COEFFICIENTS
    // ═══════════════════════════════════════════════════════════════════════════
    
    private final PIDFCoefficients translationalPIDF;
    private final PIDFCoefficients headingPIDF;
    private final PIDFCoefficients velocityPIDF;

    // ═══════════════════════════════════════════════════════════════════════════
    // THRESHOLDS & LIMITS
    // ═══════════════════════════════════════════════════════════════════════════
    
    private final double movementThreshold;      // inches - minimum movement to trigger defense
    private final double headingThreshold;       // radians - minimum rotation to trigger defense
    private final double maxCorrectionPower;     // 0.0 to 1.0 - maximum motor power for corrections
    private final double driverOverrideThreshold; // joystick deadzone
    private final double rampUpTime;             // seconds - time to reach full correction power
    private final double maxIntegral;            // anti-windup limit

    // ═══════════════════════════════════════════════════════════════════════════
    // ODOMETRY CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════════
    
    private final double ticksPerRevolution;
    private final double wheelDiameterInches;
    private final double trackWidthInches;
    private final double forwardOffsetInches;

    // Private constructor - use Builder
    private NovadConfig(Builder builder) {
        this.leftFrontMotorName = builder.leftFrontMotorName;
        this.leftRearMotorName = builder.leftRearMotorName;
        this.rightFrontMotorName = builder.rightFrontMotorName;
        this.rightRearMotorName = builder.rightRearMotorName;
        
        this.leftFrontMotorDirection = builder.leftFrontMotorDirection;
        this.leftRearMotorDirection = builder.leftRearMotorDirection;
        this.rightFrontMotorDirection = builder.rightFrontMotorDirection;
        this.rightRearMotorDirection = builder.rightRearMotorDirection;
        
        this.translationalPIDF = builder.translationalPIDF;
        this.headingPIDF = builder.headingPIDF;
        this.velocityPIDF = builder.velocityPIDF;
        
        this.movementThreshold = builder.movementThreshold;
        this.headingThreshold = builder.headingThreshold;
        this.maxCorrectionPower = builder.maxCorrectionPower;
        this.driverOverrideThreshold = builder.driverOverrideThreshold;
        this.rampUpTime = builder.rampUpTime;
        this.maxIntegral = builder.maxIntegral;
        
        this.ticksPerRevolution = builder.ticksPerRevolution;
        this.wheelDiameterInches = builder.wheelDiameterInches;
        this.trackWidthInches = builder.trackWidthInches;
        this.forwardOffsetInches = builder.forwardOffsetInches;
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // GETTERS - Motor Names
    // ═══════════════════════════════════════════════════════════════════════════
    
    public String getLeftFrontMotorName() { return leftFrontMotorName; }
    public String getLeftRearMotorName() { return leftRearMotorName; }
    public String getRightFrontMotorName() { return rightFrontMotorName; }
    public String getRightRearMotorName() { return rightRearMotorName; }

    // ═══════════════════════════════════════════════════════════════════════════
    // GETTERS - Motor Directions
    // ═══════════════════════════════════════════════════════════════════════════
    
    public MotorDirection getLeftFrontMotorDirection() { return leftFrontMotorDirection; }
    public MotorDirection getLeftRearMotorDirection() { return leftRearMotorDirection; }
    public MotorDirection getRightFrontMotorDirection() { return rightFrontMotorDirection; }
    public MotorDirection getRightRearMotorDirection() { return rightRearMotorDirection; }

    // ═══════════════════════════════════════════════════════════════════════════
    // GETTERS - PIDF Coefficients
    // ═══════════════════════════════════════════════════════════════════════════
    
    public PIDFCoefficients getTranslationalPIDF() { return translationalPIDF; }
    public PIDFCoefficients getHeadingPIDF() { return headingPIDF; }
    public PIDFCoefficients getVelocityPIDF() { return velocityPIDF; }

    // ═══════════════════════════════════════════════════════════════════════════
    // GETTERS - Thresholds & Limits
    // ═══════════════════════════════════════════════════════════════════════════
    
    public double getMovementThreshold() { return movementThreshold; }
    public double getHeadingThreshold() { return headingThreshold; }
    public double getMaxCorrectionPower() { return maxCorrectionPower; }
    public double getDriverOverrideThreshold() { return driverOverrideThreshold; }
    public double getRampUpTime() { return rampUpTime; }
    public double getMaxIntegral() { return maxIntegral; }

    // ═══════════════════════════════════════════════════════════════════════════
    // GETTERS - Odometry
    // ═══════════════════════════════════════════════════════════════════════════
    
    public double getTicksPerRevolution() { return ticksPerRevolution; }
    public double getWheelDiameterInches() { return wheelDiameterInches; }
    public double getTrackWidthInches() { return trackWidthInches; }
    public double getForwardOffsetInches() { return forwardOffsetInches; }

    // ═══════════════════════════════════════════════════════════════════════════
    // DEFAULT CONFIGURATION
    // ═══════════════════════════════════════════════════════════════════════════
    
    /**
     * Returns a default configuration suitable for most mecanum robots.
     * YOU SHOULD CUSTOMIZE THIS for your specific robot!
     */
    public static NovadConfig getDefault() {
        return new Builder()
            // Default motor names
            .leftFrontMotorName("left_front_drive")
            .leftRearMotorName("left_back_drive")
            .rightFrontMotorName("right_front_drive")
            .rightRearMotorName("right_back_drive")
            
            // Default directions (typical mecanum setup)
            .leftFrontMotorDirection(MotorDirection.REVERSE)
            .leftRearMotorDirection(MotorDirection.REVERSE)
            .rightFrontMotorDirection(MotorDirection.FORWARD)
            .rightRearMotorDirection(MotorDirection.FORWARD)
            
            // Default PIDF (TUNE THESE!)
            .translationalPIDFCoefficients(new PIDFCoefficients(0.046, 0, 0.01, 0.02))
            .headingPIDFCoefficients(new PIDFCoefficients(0.67, 0, 0.006, 0.02))
            .velocityPIDFCoefficients(new PIDFCoefficients(0.02, 0, 0.001, 0.01))
            
            // Default thresholds
            .movementThreshold(0.5)
            .headingThreshold(Math.toRadians(2.0))
            .maxCorrectionPower(0.8)
            .driverOverrideThreshold(0.1)
            .rampUpTime(0.3)
            .maxIntegral(1.0)
            
            // Default odometry (48mm wheel, REV encoder)
            .ticksPerRevolution(8192)
            .wheelDiameterInches(1.89)
            .trackWidthInches(14.0)
            .forwardOffsetInches(6.0)
            
            .build();
    }

    // ═══════════════════════════════════════════════════════════════════════════
    // BUILDER CLASS
    // ═══════════════════════════════════════════════════════════════════════════
    
    public static class Builder {
        // Motor names
        private String leftFrontMotorName = "left_front_drive";
        private String leftRearMotorName = "left_back_drive";
        private String rightFrontMotorName = "right_front_drive";
        private String rightRearMotorName = "right_back_drive";
        
        // Motor directions
        private MotorDirection leftFrontMotorDirection = MotorDirection.REVERSE;
        private MotorDirection leftRearMotorDirection = MotorDirection.REVERSE;
        private MotorDirection rightFrontMotorDirection = MotorDirection.FORWARD;
        private MotorDirection rightRearMotorDirection = MotorDirection.FORWARD;
        
        // PIDF coefficients
        private PIDFCoefficients translationalPIDF = new PIDFCoefficients(0.046, 0, 0.01, 0.02);
        private PIDFCoefficients headingPIDF = new PIDFCoefficients(0.67, 0, 0.006, 0.02);
        private PIDFCoefficients velocityPIDF = new PIDFCoefficients(0.02, 0, 0.001, 0.01);
        
        // Thresholds
        private double movementThreshold = 0.5;
        private double headingThreshold = Math.toRadians(2.0);
        private double maxCorrectionPower = 0.8;
        private double driverOverrideThreshold = 0.1;
        private double rampUpTime = 0.3;
        private double maxIntegral = 1.0;
        
        // Odometry
        private double ticksPerRevolution = 8192;
        private double wheelDiameterInches = 1.89;
        private double trackWidthInches = 14.0;
        private double forwardOffsetInches = 6.0;

        // ═══════════════════════════════════════════════════════════════════════
        // MOTOR NAME SETTERS
        // ═══════════════════════════════════════════════════════════════════════
        
        public Builder leftFrontMotorName(String name) {
            this.leftFrontMotorName = name;
            return this;
        }
        
        public Builder leftRearMotorName(String name) {
            this.leftRearMotorName = name;
            return this;
        }
        
        public Builder rightFrontMotorName(String name) {
            this.rightFrontMotorName = name;
            return this;
        }
        
        public Builder rightRearMotorName(String name) {
            this.rightRearMotorName = name;
            return this;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // MOTOR DIRECTION SETTERS
        // ═══════════════════════════════════════════════════════════════════════
        
        public Builder leftFrontMotorDirection(MotorDirection direction) {
            this.leftFrontMotorDirection = direction;
            return this;
        }
        
        public Builder leftRearMotorDirection(MotorDirection direction) {
            this.leftRearMotorDirection = direction;
            return this;
        }
        
        public Builder rightFrontMotorDirection(MotorDirection direction) {
            this.rightFrontMotorDirection = direction;
            return this;
        }
        
        public Builder rightRearMotorDirection(MotorDirection direction) {
            this.rightRearMotorDirection = direction;
            return this;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // PIDF COEFFICIENT SETTERS
        // ═══════════════════════════════════════════════════════════════════════
        
        public Builder translationalPIDFCoefficients(PIDFCoefficients pidf) {
            this.translationalPIDF = pidf;
            return this;
        }
        
        public Builder headingPIDFCoefficients(PIDFCoefficients pidf) {
            this.headingPIDF = pidf;
            return this;
        }
        
        public Builder velocityPIDFCoefficients(PIDFCoefficients pidf) {
            this.velocityPIDF = pidf;
            return this;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // THRESHOLD SETTERS
        // ═══════════════════════════════════════════════════════════════════════
        
        public Builder movementThreshold(double threshold) {
            this.movementThreshold = threshold;
            return this;
        }
        
        public Builder headingThreshold(double threshold) {
            this.headingThreshold = threshold;
            return this;
        }
        
        public Builder maxCorrectionPower(double power) {
            this.maxCorrectionPower = power;
            return this;
        }
        
        public Builder driverOverrideThreshold(double threshold) {
            this.driverOverrideThreshold = threshold;
            return this;
        }
        
        public Builder rampUpTime(double seconds) {
            this.rampUpTime = seconds;
            return this;
        }
        
        public Builder maxIntegral(double max) {
            this.maxIntegral = max;
            return this;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // ODOMETRY SETTERS
        // ═══════════════════════════════════════════════════════════════════════
        
        public Builder ticksPerRevolution(double ticks) {
            this.ticksPerRevolution = ticks;
            return this;
        }
        
        public Builder wheelDiameterInches(double diameter) {
            this.wheelDiameterInches = diameter;
            return this;
        }
        
        public Builder trackWidthInches(double width) {
            this.trackWidthInches = width;
            return this;
        }
        
        public Builder forwardOffsetInches(double offset) {
            this.forwardOffsetInches = offset;
            return this;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // BUILD
        // ═══════════════════════════════════════════════════════════════════════
        
        public NovadConfig build() {
            return new NovadConfig(this);
        }
    }
}
