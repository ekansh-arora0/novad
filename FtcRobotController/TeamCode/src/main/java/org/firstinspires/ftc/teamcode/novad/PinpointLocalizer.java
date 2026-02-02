package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

/**
 * Pinpoint Odometry Localizer
 * 
 * Uses the FTC SDK's built-in GoBildaPinpointDriver to get robot pose.
 * This is the easiest odometry option - just plug in the Pinpoint and go!
 */
public class PinpointLocalizer implements Localizer {
    
    private final GoBildaPinpointDriver pinpoint;
    private Pose2d currentPose = new Pose2d();
    private double headingOffset = 0;

    /**
     * Create a Pinpoint localizer
     * 
     * @param hardwareMap The hardware map from OpMode
     * @param deviceName The configured name of the Pinpoint (usually "pinpoint")
     */
    public PinpointLocalizer(HardwareMap hardwareMap, String deviceName) {
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        
        // Reset position to 0,0
        pinpoint.resetPosAndIMU();
    }

    @Override
    public void update() {
        // Read latest data from Pinpoint
        pinpoint.update();
        
        // Get position in inches and heading in radians
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
     * Recalibrate the IMU (robot should be stationary)
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
