package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Three-Wheel Dead Wheel Localizer
 * 
 * Uses three dead wheel encoders to track robot position:
 * - Two parallel wheels (left and right) for forward movement and turning
 * - One perpendicular wheel (center) for strafing
 * 
 * Based on pose exponential method for accurate tracking.
 */
public class ThreeWheelLocalizer implements Localizer {
    
    private final DcMotor leftEncoder, rightEncoder, centerEncoder;
    private final double ticksToInches;
    private final double trackWidth;
    private final double centerOffset;
    private final int leftDirection, rightDirection, centerDirection;
    
    private Pose2d currentPose = new Pose2d();
    private int lastLeftTicks = 0, lastRightTicks = 0, lastCenterTicks = 0;
    private boolean initialized = false;

    /**
     * Create a three-wheel localizer
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param leftName Motor name for left encoder port
     * @param rightName Motor name for right encoder port
     * @param centerName Motor name for center encoder port
     * @param ticksToInches Encoder ticks to inches conversion factor
     * @param trackWidth Distance between left and right wheels (inches)
     * @param centerOffset Distance from center to strafe wheel (inches, + = forward)
     * @param leftReversed Reverse left encoder direction
     * @param rightReversed Reverse right encoder direction
     * @param centerReversed Reverse center encoder direction
     */
    public ThreeWheelLocalizer(
            HardwareMap hardwareMap,
            String leftName, String rightName, String centerName,
            double ticksToInches, double trackWidth, double centerOffset,
            boolean leftReversed, boolean rightReversed, boolean centerReversed) {
        
        // Get encoder motor references (we only read encoders, not drive motors)
        leftEncoder = hardwareMap.get(DcMotor.class, leftName);
        rightEncoder = hardwareMap.get(DcMotor.class, rightName);
        centerEncoder = hardwareMap.get(DcMotor.class, centerName);
        
        // Store configuration
        this.ticksToInches = ticksToInches;
        this.trackWidth = trackWidth;
        this.centerOffset = centerOffset;
        this.leftDirection = leftReversed ? -1 : 1;
        this.rightDirection = rightReversed ? -1 : 1;
        this.centerDirection = centerReversed ? -1 : 1;
        
        // Reset encoders
        leftEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        leftEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rightEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        centerEncoder.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    @Override
    public void update() {
        // Read current encoder values
        int leftTicks = leftEncoder.getCurrentPosition() * leftDirection;
        int rightTicks = rightEncoder.getCurrentPosition() * rightDirection;
        int centerTicks = centerEncoder.getCurrentPosition() * centerDirection;
        
        if (!initialized) {
            lastLeftTicks = leftTicks;
            lastRightTicks = rightTicks;
            lastCenterTicks = centerTicks;
            initialized = true;
            return;
        }
        
        // Calculate deltas in ticks
        int deltaLeft = leftTicks - lastLeftTicks;
        int deltaRight = rightTicks - lastRightTicks;
        int deltaCenter = centerTicks - lastCenterTicks;
        
        // Convert to inches
        double leftInches = deltaLeft * ticksToInches;
        double rightInches = deltaRight * ticksToInches;
        double centerInches = deltaCenter * ticksToInches;
        
        // Calculate robot-relative movements using pose exponential
        double deltaTheta = (rightInches - leftInches) / trackWidth;
        double deltaForward = (leftInches + rightInches) / 2.0;
        double deltaStrafe = centerInches - (centerOffset * deltaTheta);
        
        // Update pose using rotation matrix (field-centric integration)
        double avgHeading = currentPose.heading + deltaTheta / 2.0;
        double cos = Math.cos(avgHeading);
        double sin = Math.sin(avgHeading);
        
        currentPose = new Pose2d(
            currentPose.x + deltaForward * cos - deltaStrafe * sin,
            currentPose.y + deltaForward * sin + deltaStrafe * cos,
            Pose2d.normalizeAngle(currentPose.heading + deltaTheta)
        );
        
        // Save for next iteration
        lastLeftTicks = leftTicks;
        lastRightTicks = rightTicks;
        lastCenterTicks = centerTicks;
    }

    @Override
    public Pose2d getPose() {
        return currentPose;
    }

    @Override
    public void setPose(Pose2d pose) {
        currentPose = pose;
    }

    @Override
    public void resetHeading() {
        currentPose = new Pose2d(currentPose.x, currentPose.y, 0);
    }
}
