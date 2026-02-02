package org.firstinspires.ftc.teamcode.novad.adapters;package org.firstinspires.ftc.teamcode.novad.adapters;



import com.qualcomm.robotcore.hardware.DcMotor;import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;

import com.qualcomm.robotcore.hardware.HardwareMap;import org.firstinspires.ftc.teamcode.novad.util.Vector2D;



import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;/**

import org.firstinspires.ftc.teamcode.novad.util.Vector2D; * Generic Three-Wheel Odometry Implementation

 * 

/** * Use this class if you want Novad to manage your odometry directly

 * Three-Wheel Odometry Implementation * using three dead wheel encoders.

 *  * 

 * IMPORTANT: Dead wheel encoders plug into MOTOR ENCODER PORTS! * Wheel configuration:

 * You specify which motor port each encoder is plugged into. * - Left wheel: parallel to left side of robot

 *  * - Right wheel: parallel to right side of robot  

 * Wheel configuration: * - Center wheel: perpendicular (for strafe)

 * - Left wheel: parallel to left side of robot * 

 * - Right wheel: parallel to right side of robot   * Example:

 * - Center wheel: perpendicular (for strafe) * <pre>

 *  * ThreeWheelOdometry odometry = new ThreeWheelOdometry(

 * Example using NovadConstants: *     () -> leftEncoder.getCurrentPosition(),

 * <pre> *     () -> rightEncoder.getCurrentPosition(),

 * ThreeWheelOdometry odometry = new ThreeWheelOdometry( *     () -> centerEncoder.getCurrentPosition(),

 *     hardwareMap, *     2.0,    // wheel diameter in inches

 *     NovadConstants.LEFT_ENCODER_MOTOR_PORT, *     8192,   // encoder ticks per revolution

 *     NovadConstants.RIGHT_ENCODER_MOTOR_PORT, *     12.0,   // track width (distance between left and right wheels)

 *     NovadConstants.CENTER_ENCODER_MOTOR_PORT, *     6.0     // forward offset (distance from center to strafe wheel)

 *     NovadConstants.LEFT_ENCODER_REVERSED, * );

 *     NovadConstants.RIGHT_ENCODER_REVERSED, * </pre>

 *     NovadConstants.CENTER_ENCODER_REVERSED, * 

 *     NovadConstants.DEAD_WHEEL_DIAMETER_INCHES, * @author Novad Defense Library

 *     NovadConstants.DEAD_WHEEL_TICKS_PER_REV, * @version 1.0.0

 *     NovadConstants.TRACK_WIDTH_INCHES, */

 *     NovadConstants.FORWARD_OFFSET_INCHESpublic class ThreeWheelOdometry implements NovadOdometry {

 * );

 * </pre>    /**

 *      * Functional interface for reading encoder position

 * @author Novad Defense Library     */

 * @version 1.0.0    @FunctionalInterface

 */    public interface EncoderReader {

public class ThreeWheelOdometry implements NovadOdometry {        int getPosition();

    }

    // Motors that have encoders plugged into their encoder ports

    private final DcMotor leftEncoderMotor;    // Encoder readers

    private final DcMotor rightEncoderMotor;    private final EncoderReader leftEncoder;

    private final DcMotor centerEncoderMotor;    private final EncoderReader rightEncoder;

        private final EncoderReader centerEncoder;

    // Direction multipliers

    private final int leftDirection;    // Configuration

    private final int rightDirection;    private final double inchesPerTick;

    private final int centerDirection;    private final double trackWidth;

    private final double forwardOffset;

    // Configuration

    private final double inchesPerTick;    // State

    private final double trackWidth;    private double x, y, heading;

    private final double forwardOffset;    private double vx, vy, omega;



    // State    // Previous encoder values

    private double x, y, heading;    private int prevLeftTicks, prevRightTicks, prevCenterTicks;

    private double vx, vy, omega;    private long prevTime;

    

    // Acceleration tracking for predictive defense    /**

    private double ax, ay;     * Create a three-wheel odometry system

    private double lastVx, lastVy;     * 

     * @param leftEncoder Left encoder position reader

    // Previous encoder values     * @param rightEncoder Right encoder position reader

    private int prevLeftTicks, prevRightTicks, prevCenterTicks;     * @param centerEncoder Center/strafe encoder position reader

    private long prevTime;     * @param wheelDiameter Wheel diameter in inches

     * @param ticksPerRevolution Encoder ticks per full wheel revolution

    /**     * @param trackWidth Distance between left and right wheels in inches

     * Create a three-wheel odometry system using motor encoder ports     * @param forwardOffset Distance from robot center to center wheel in inches

     *      */

     * @param hardwareMap Robot hardware map    public ThreeWheelOdometry(

     * @param leftEncoderMotorName Name of motor with left encoder plugged in            EncoderReader leftEncoder,

     * @param rightEncoderMotorName Name of motor with right encoder plugged in            EncoderReader rightEncoder,

     * @param centerEncoderMotorName Name of motor with center encoder plugged in            EncoderReader centerEncoder,

     * @param leftReversed True if left encoder should be reversed            double wheelDiameter,

     * @param rightReversed True if right encoder should be reversed            int ticksPerRevolution,

     * @param centerReversed True if center encoder should be reversed            double trackWidth,

     * @param wheelDiameter Wheel diameter in inches            double forwardOffset) {

     * @param ticksPerRevolution Encoder ticks per full wheel revolution        

     * @param trackWidth Distance between left and right wheels in inches        this.leftEncoder = leftEncoder;

     * @param forwardOffset Distance from robot center to center wheel in inches        this.rightEncoder = rightEncoder;

     */        this.centerEncoder = centerEncoder;

    public ThreeWheelOdometry(

            HardwareMap hardwareMap,        // Calculate inches per encoder tick

            String leftEncoderMotorName,        this.inchesPerTick = (wheelDiameter * Math.PI) / ticksPerRevolution;

            String rightEncoderMotorName,        this.trackWidth = trackWidth;

            String centerEncoderMotorName,        this.forwardOffset = forwardOffset;

            boolean leftReversed,

            boolean rightReversed,        // Initialize state

            boolean centerReversed,        this.x = 0;

            double wheelDiameter,        this.y = 0;

            int ticksPerRevolution,        this.heading = 0;

            double trackWidth,        this.vx = 0;

            double forwardOffset) {        this.vy = 0;

                this.omega = 0;

        // Get motor references (we only use them for encoder readings)

        this.leftEncoderMotor = hardwareMap.get(DcMotor.class, leftEncoderMotorName);        // Initialize previous values

        this.rightEncoderMotor = hardwareMap.get(DcMotor.class, rightEncoderMotorName);        this.prevLeftTicks = leftEncoder.getPosition();

        this.centerEncoderMotor = hardwareMap.get(DcMotor.class, centerEncoderMotorName);        this.prevRightTicks = rightEncoder.getPosition();

                this.prevCenterTicks = centerEncoder.getPosition();

        // Direction multipliers        this.prevTime = System.nanoTime();

        this.leftDirection = leftReversed ? -1 : 1;    }

        this.rightDirection = rightReversed ? -1 : 1;

        this.centerDirection = centerReversed ? -1 : 1;    @Override

    public void update() {

        // Calculate inches per encoder tick        // Read current encoder positions

        this.inchesPerTick = (wheelDiameter * Math.PI) / ticksPerRevolution;        int leftTicks = leftEncoder.getPosition();

        this.trackWidth = trackWidth;        int rightTicks = rightEncoder.getPosition();

        this.forwardOffset = forwardOffset;        int centerTicks = centerEncoder.getPosition();

        long currentTime = System.nanoTime();

        // Initialize state

        this.x = 0;        // Calculate deltas

        this.y = 0;        int deltaLeft = leftTicks - prevLeftTicks;

        this.heading = 0;        int deltaRight = rightTicks - prevRightTicks;

        this.vx = 0;        int deltaCenter = centerTicks - prevCenterTicks;

        this.vy = 0;        double dt = (currentTime - prevTime) / 1_000_000_000.0;

        this.omega = 0;

        this.ax = 0;        // Convert to inches

        this.ay = 0;        double leftDist = deltaLeft * inchesPerTick;

        double rightDist = deltaRight * inchesPerTick;

        // Initialize previous values        double centerDist = deltaCenter * inchesPerTick;

        this.prevLeftTicks = leftEncoderMotor.getCurrentPosition() * leftDirection;

        this.prevRightTicks = rightEncoderMotor.getCurrentPosition() * rightDirection;        // Calculate heading change

        this.prevCenterTicks = centerEncoderMotor.getCurrentPosition() * centerDirection;        double deltaHeading = (rightDist - leftDist) / trackWidth;

        this.prevTime = System.nanoTime();

    }        // Calculate forward and strafe movement

            double deltaForward = (leftDist + rightDist) / 2.0;

    /**        double deltaStrafe = centerDist - (forwardOffset * deltaHeading);

     * Simplified constructor using functional interfaces (for custom encoder sources)

     */        // Update position using arc integration

    public ThreeWheelOdometry(        double avgHeading = heading + deltaHeading / 2.0;

            EncoderReader leftEncoder,        double cos = Math.cos(avgHeading);

            EncoderReader rightEncoder,        double sin = Math.sin(avgHeading);

            EncoderReader centerEncoder,

            double wheelDiameter,        x += deltaForward * cos - deltaStrafe * sin;

            int ticksPerRevolution,        y += deltaForward * sin + deltaStrafe * cos;

            double trackWidth,        heading += deltaHeading;

            double forwardOffset) {

                // Normalize heading to [-π, π]

        // Store as functional interface - use wrapper motors        while (heading > Math.PI) heading -= 2 * Math.PI;

        this.leftEncoderMotor = null;        while (heading < -Math.PI) heading += 2 * Math.PI;

        this.rightEncoderMotor = null;

        this.centerEncoderMotor = null;        // Calculate velocities

                if (dt > 0.001) {

        this.leftDirection = 1;            vx = (deltaForward * cos - deltaStrafe * sin) / dt;

        this.rightDirection = 1;            vy = (deltaForward * sin + deltaStrafe * cos) / dt;

        this.centerDirection = 1;            omega = deltaHeading / dt;

                }

        // Store readers for functional interface version

        this.leftReader = leftEncoder;        // Save current values

        this.rightReader = rightEncoder;        prevLeftTicks = leftTicks;

        this.centerReader = centerEncoder;        prevRightTicks = rightTicks;

        prevCenterTicks = centerTicks;

        // Calculate inches per encoder tick        prevTime = currentTime;

        this.inchesPerTick = (wheelDiameter * Math.PI) / ticksPerRevolution;    }

        this.trackWidth = trackWidth;

        this.forwardOffset = forwardOffset;    @Override

    public Vector2D getPosition() {

        // Initialize state        return new Vector2D(x, y);

        this.x = 0;    }

        this.y = 0;

        this.heading = 0;    @Override

        this.vx = 0;    public double getX() {

        this.vy = 0;        return x;

        this.omega = 0;    }

        this.ax = 0;

        this.ay = 0;    @Override

    public double getY() {

        // Initialize previous values        return y;

        this.prevLeftTicks = leftEncoder.getPosition();    }

        this.prevRightTicks = rightEncoder.getPosition();

        this.prevCenterTicks = centerEncoder.getPosition();    @Override

        this.prevTime = System.nanoTime();    public double getHeading() {

    }        return heading;

        }

    // Optional functional interface readers

    private EncoderReader leftReader, rightReader, centerReader;    @Override

        public Vector2D getVelocity() {

    /**        return new Vector2D(vx, vy);

     * Functional interface for reading encoder position    }

     */

    @FunctionalInterface    @Override

    public interface EncoderReader {    public double getXVelocity() {

        int getPosition();        return vx;

    }    }



    @Override    @Override

    public void update() {    public double getYVelocity() {

        // Read current encoder positions        return vy;

        int leftTicks, rightTicks, centerTicks;    }

        

        if (leftEncoderMotor != null) {    @Override

            // Using motor encoder ports    public double getAngularVelocity() {

            leftTicks = leftEncoderMotor.getCurrentPosition() * leftDirection;        return omega;

            rightTicks = rightEncoderMotor.getCurrentPosition() * rightDirection;    }

            centerTicks = centerEncoderMotor.getCurrentPosition() * centerDirection;

        } else {    @Override

            // Using functional interface readers    public void setPose(double x, double y, double heading) {

            leftTicks = leftReader.getPosition();        this.x = x;

            rightTicks = rightReader.getPosition();        this.y = y;

            centerTicks = centerReader.getPosition();        this.heading = heading;

        }    }

        

        long currentTime = System.nanoTime();    /**

     * Get the inches per encoder tick conversion factor

        // Calculate deltas     * @return Inches per tick

        int deltaLeft = leftTicks - prevLeftTicks;     */

        int deltaRight = rightTicks - prevRightTicks;    public double getInchesPerTick() {

        int deltaCenter = centerTicks - prevCenterTicks;        return inchesPerTick;

        double dt = (currentTime - prevTime) / 1_000_000_000.0;    }



        // Convert to inches    /**

        double leftDist = deltaLeft * inchesPerTick;     * Get the track width

        double rightDist = deltaRight * inchesPerTick;     * @return Track width in inches

        double centerDist = deltaCenter * inchesPerTick;     */

    public double getTrackWidth() {

        // Calculate heading change        return trackWidth;

        double deltaHeading = (rightDist - leftDist) / trackWidth;    }



        // Calculate forward and strafe movement    /**

        double deltaForward = (leftDist + rightDist) / 2.0;     * Get the forward offset

        double deltaStrafe = centerDist - (forwardOffset * deltaHeading);     * @return Forward offset in inches

     */

        // Update position using arc integration    public double getForwardOffset() {

        double avgHeading = heading + deltaHeading / 2.0;        return forwardOffset;

        double cos = Math.cos(avgHeading);    }

        double sin = Math.sin(avgHeading);}


        x += deltaForward * cos - deltaStrafe * sin;
        y += deltaForward * sin + deltaStrafe * cos;
        heading += deltaHeading;

        // Normalize heading to [-π, π]
        while (heading > Math.PI) heading -= 2 * Math.PI;
        while (heading < -Math.PI) heading += 2 * Math.PI;

        // Calculate velocities and accelerations
        if (dt > 0.001) {
            double newVx = (deltaForward * cos - deltaStrafe * sin) / dt;
            double newVy = (deltaForward * sin + deltaStrafe * cos) / dt;
            omega = deltaHeading / dt;
            
            // Calculate accelerations (for predictive defense)
            ax = (newVx - lastVx) / dt;
            ay = (newVy - lastVy) / dt;
            
            lastVx = vx;
            lastVy = vy;
            vx = newVx;
            vy = newVy;
        }

        // Save current values
        prevLeftTicks = leftTicks;
        prevRightTicks = rightTicks;
        prevCenterTicks = centerTicks;
        prevTime = currentTime;
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
     * Get the inches per encoder tick conversion factor
     */
    public double getInchesPerTick() {
        return inchesPerTick;
    }

    /**
     * Get the track width
     */
    public double getTrackWidth() {
        return trackWidth;
    }

    /**
     * Get the forward offset
     */
    public double getForwardOffset() {
        return forwardOffset;
    }
}
