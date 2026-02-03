package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

/**
 * Pinpoint localizer configuration constants
 */
public class PinpointConstants {
    private String hardwareMapName = "pinpoint";
    private GoBildaPinpointDriver.GoBildaOdometryPods encoderResolution = 
            GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD;
    private double customEncoderResolution = 0;
    private GoBildaPinpointDriver.EncoderDirection forwardEncoderDirection = 
            GoBildaPinpointDriver.EncoderDirection.FORWARD;
    private GoBildaPinpointDriver.EncoderDirection strafeEncoderDirection = 
            GoBildaPinpointDriver.EncoderDirection.FORWARD;

    public PinpointConstants hardwareMapName(String name) {
        this.hardwareMapName = name;
        return this;
    }

    public PinpointConstants encoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods resolution) {
        this.encoderResolution = resolution;
        this.customEncoderResolution = 0;
        return this;
    }

    public PinpointConstants customEncoderResolution(double ticksPerMm) {
        this.customEncoderResolution = ticksPerMm;
        this.encoderResolution = null;
        return this;
    }

    public PinpointConstants forwardEncoderDirection(GoBildaPinpointDriver.EncoderDirection direction) {
        this.forwardEncoderDirection = direction;
        return this;
    }

    public PinpointConstants strafeEncoderDirection(GoBildaPinpointDriver.EncoderDirection direction) {
        this.strafeEncoderDirection = direction;
        return this;
    }

    // Getters
    public String getHardwareMapName() { return hardwareMapName; }
    public GoBildaPinpointDriver.GoBildaOdometryPods getEncoderResolution() { return encoderResolution; }
    public double getCustomEncoderResolution() { return customEncoderResolution; }
    public GoBildaPinpointDriver.EncoderDirection getForwardEncoderDirection() { return forwardEncoderDirection; }
    public GoBildaPinpointDriver.EncoderDirection getStrafeEncoderDirection() { return strafeEncoderDirection; }
    public boolean isUsingCustomResolution() { return customEncoderResolution > 0; }
}
