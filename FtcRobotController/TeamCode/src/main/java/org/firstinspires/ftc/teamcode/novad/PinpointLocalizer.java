package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.NovadSetup;

/**
 * Pinpoint Odometry Localizer
 * 
 * Uses goBilda Pinpoint Odometry Computer.
 * Requires configuration in NovadSetup.java:
 * - PINPOINT_X_OFFSET: X offset of Pinpoint from robot center (mm)
 * - PINPOINT_Y_OFFSET: Y offset of Pinpoint from robot center (mm)
 * - PINPOINT_ENCODER_RESOLUTION: Which encoder pods you're using
 */
public class PinpointLocalizer implements Localizer {
    
    private final GoBildaPinpointDriver pinpoint;
    private Pose2d currentPose = new Pose2d();
    private double headingOffset = 0;

    public PinpointLocalizer(HardwareMap hardwareMap, String deviceName) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        
        // Configure encoder resolution based on NovadSetup
        // goBilda odometry pods come in two resolutions
        if (NovadSetup.PINPOINT_USE_GOBILDA_PODS) {
            pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);
        } else {
            // Custom encoder - set ticks per mm
            pinpoint.setEncoderResolution(NovadSetup.PINPOINT_TICKS_PER_MM);
        }
        
        // Set pod offsets (distance from robot center in mm)
        // X = forward/backward offset, Y = left/right offset
        pinpoint.setOffsets(NovadSetup.PINPOINT_X_OFFSET, NovadSetup.PINPOINT_Y_OFFSET);
        
        // Set encoder directions if needed
        pinpoint.setEncoderDirections(
            NovadSetup.PINPOINT_X_REVERSED 
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED 
                : GoBildaPinpointDriver.EncoderDirection.FORWARD,
            NovadSetup.PINPOINT_Y_REVERSED 
                ? GoBildaPinpointDriver.EncoderDirection.REVERSED 
                : GoBildaPinpointDriver.EncoderDirection.FORWARD
        );
        
        // Reset position to 0,0
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void update() {
        pinpoint.update();
        
        Pose2D pose = pinpoint.getPosition();
        
        currentPose = new Pose2d(
            pose.getX(DistanceUnit.INCH),
            pose.getY(DistanceUnit.INCH),
            Pose2d.normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset)
        );
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        pinpoint.setPosition(new Pose2D(
            DistanceUnit.INCH, 
            pose.x, 
            pose.y, 
            AngleUnit.RADIANS, 
            pose.heading
        ));
        headingOffset = 0;
        currentPose = pose;
    }

    @Override
    public void resetHeading() {
        headingOffset = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }
    
    /**
     * Recalibrate the IMU (robot must be stationary)
     */
    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }
    
    /**
     * Get device status for debugging
     */
    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }
}
