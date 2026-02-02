package org.firstinspires.ftc.teamcode.novad.adapters;

import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;
import org.firstinspires.ftc.teamcode.novad.util.Vector2D;

/**
 * Pinpoint Odometry Adapter - For GoBilda Pinpoint Odometry Computer
 * 
 * The Pinpoint is a dedicated odometry computer that handles all position
 * calculations on-device, providing highly accurate position tracking.
 * 
 * WIRING:
 * - Connect Pinpoint to I2C port on Control Hub
 * - Connect two odometry pods to Pinpoint's encoder ports
 * 
 * CONFIGURATION:
 * - Add "GoBilda Pinpoint Odometry Computer" as I2C device in Robot Config
 * - Name it "pinpoint" (or change PINPOINT_DEVICE_NAME in NovadConstants)
 * 
 * Example:
 * <pre>
 * PinpointOdometry odometry = new PinpointOdometry(hardwareMap);
 * Novad novad = new Novad(odometry, drivetrain);
 * </pre>
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class PinpointOdometry implements NovadOdometry {

    // GoBilda Pinpoint driver class (from goBILDA SDK)
    private final GoBildaPinpointDriver pinpoint;
    
    // State
    private double x, y, heading;
    private double vx, vy, omega;
    private double lastX, lastY, lastHeading;
    private long lastTime;
    
    // Acceleration tracking for predictive defense
    private double ax, ay;
    private double lastVx, lastVy;

    /**
     * Create Pinpoint odometry with default device name "pinpoint"
     */
    public PinpointOdometry(HardwareMap hardwareMap) {
        this(hardwareMap, "pinpoint", 0, 0, 48, 8192);
    }
    
    /**
     * Create Pinpoint odometry with just device name (most common usage)
     */
    public PinpointOdometry(HardwareMap hardwareMap, String deviceName) {
        this(hardwareMap, deviceName, 0, 0, 48, 8192);
    }

    /**
     * Create Pinpoint odometry with custom configuration
     * 
     * @param hardwareMap Robot hardware map
     * @param deviceName I2C device name from robot configuration
     * @param xOffsetMm X offset from robot center to Pinpoint (mm, positive = forward)
     * @param yOffsetMm Y offset from robot center to Pinpoint (mm, positive = left)
     * @param wheelDiameterMm Odometry pod wheel diameter in mm
     * @param encoderResolution Encoder ticks per revolution
     */
    public PinpointOdometry(HardwareMap hardwareMap, String deviceName,
                            double xOffsetMm, double yOffsetMm,
                            double wheelDiameterMm, double encoderResolution) {
        this.pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, deviceName);
        
        // Configure Pinpoint
        pinpoint.setOffsets(xOffsetMm, yOffsetMm);
        pinpoint.setEncoderResolution(encoderResolution);
        
        // Reset position
        pinpoint.resetPosAndIMU();
        
        // Initialize state
        this.x = 0;
        this.y = 0;
        this.heading = 0;
        this.vx = 0;
        this.vy = 0;
        this.omega = 0;
        this.ax = 0;
        this.ay = 0;
        this.lastTime = System.nanoTime();
    }

    @Override
    public void update() {
        // Update Pinpoint readings
        pinpoint.update();
        
        // Get position from Pinpoint (in mm, convert to inches)
        Pose2D pose = pinpoint.getPosition();
        double newX = pose.getX(DistanceUnit.INCH);
        double newY = pose.getY(DistanceUnit.INCH);
        double newHeading = pose.getHeading(AngleUnit.RADIANS);
        
        // Calculate time delta
        long currentTime = System.nanoTime();
        double dt = (currentTime - lastTime) / 1_000_000_000.0; // seconds
        
        if (dt > 0.001) { // Avoid division by zero
            // Calculate velocities
            double newVx = (newX - lastX) / dt;
            double newVy = (newY - lastY) / dt;
            this.omega = (newHeading - lastHeading) / dt;
            
            // Calculate accelerations (for predictive defense)
            this.ax = (newVx - lastVx) / dt;
            this.ay = (newVy - lastVy) / dt;
            
            // Store for next iteration
            this.lastVx = this.vx;
            this.lastVy = this.vy;
            this.vx = newVx;
            this.vy = newVy;
        }
        
        // Update position
        this.lastX = this.x;
        this.lastY = this.y;
        this.lastHeading = this.heading;
        this.x = newX;
        this.y = newY;
        this.heading = newHeading;
        this.lastTime = currentTime;
    }

    @Override
    public Vector2D getPosition() {
        return new Vector2D(x, y);
    }

    @Override
    public double getHeading() {
        return heading;
    }

    @Override
    public Vector2D getVelocity() {
        return new Vector2D(vx, vy);
    }

    @Override
    public void setPose(double x, double y, double heading) {
        this.x = x;
        this.y = y;
        this.heading = heading;
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.RADIANS, heading));
    }
    
    /**
     * Get current acceleration (for predictive defense)
     * @return Acceleration vector in inches/second²
     */
    public Vector2D getAcceleration() {
        return new Vector2D(ax, ay);
    }
    
    /**
     * Get angular velocity
     * @return Angular velocity in radians/second
     */
    public double getAngularVelocity() {
        return omega;
    }
    
    /**
     * Recalibrate the IMU. Robot should be stationary when called.
     */
    public void recalibrateIMU() {
        pinpoint.recalibrateIMU();
    }
    
    /**
     * Reset position to (0, 0, 0)
     */
    public void resetPosition() {
        pinpoint.resetPosAndIMU();
        this.x = 0;
        this.y = 0;
        this.heading = 0;
        this.vx = 0;
        this.vy = 0;
        this.omega = 0;
    }
    
    /**
     * Get Pinpoint device status
     */
    public GoBildaPinpointDriver.DeviceStatus getStatus() {
        return pinpoint.getDeviceStatus();
    }
}

/**
 * GoBilda Pinpoint Driver Interface
 * 
 * This is a placeholder interface. In the actual FTC SDK, this class is provided
 * by GoBilda's driver. Teams should use the official GoBildaPinpointDriver class.
 * 
 * For now, this allows the code to compile. Replace with actual import:
 * import com.gobilda.pinpoint.GoBildaPinpointDriver;
 */
interface GoBildaPinpointDriver {
    void setOffsets(double xMm, double yMm);
    void setEncoderResolution(double resolution);
    void resetPosAndIMU();
    void recalibrateIMU();
    void update();
    Pose2D getPosition();
    void setPosition(Pose2D pose);
    DeviceStatus getDeviceStatus();
    
    enum DeviceStatus {
        READY,
        CALIBRATING,
        NOT_READY,
        FAULT
    }
}
