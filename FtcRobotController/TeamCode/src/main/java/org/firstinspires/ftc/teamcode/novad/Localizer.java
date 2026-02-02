package org.firstinspires.ftc.teamcode.novad;

/**
 * Localizer Interface
 * 
 * Provides robot pose estimation. Implementations exist for:
 * - GoBilda Pinpoint (PinpointLocalizer)
 * - Three Dead Wheels (ThreeWheelLocalizer)
 */
public interface Localizer {
    
    /**
     * Update the pose estimate.
     * Call this every loop iteration.
     */
    void update();
    
    /**
     * Get the current estimated pose.
     * @return Current robot pose (x, y in inches, heading in radians)
     */
    Pose2d getPose();
    
    /**
     * Set the current pose (useful for resetting or setting start position)
     * @param pose New pose to set
     */
    void setPose(Pose2d pose);
    
    /**
     * Reset heading to zero (useful for field-centric driving)
     */
    void resetHeading();
}
