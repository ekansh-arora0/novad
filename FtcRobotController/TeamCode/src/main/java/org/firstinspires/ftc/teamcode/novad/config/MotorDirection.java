package org.firstinspires.ftc.teamcode.novad.config;

/**
 * Motor direction enum (mirrors FTC SDK's DcMotorSimple.Direction)
 * Used for configuration without requiring SDK dependency
 */
public enum MotorDirection {
    FORWARD(1.0),
    REVERSE(-1.0);
    
    private final double multiplier;
    
    MotorDirection(double multiplier) {
        this.multiplier = multiplier;
    }
    
    public double getMultiplier() {
        return multiplier;
    }
}
