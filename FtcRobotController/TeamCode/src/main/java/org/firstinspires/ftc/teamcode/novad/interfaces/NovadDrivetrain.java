package org.firstinspires.ftc.teamcode.novad.interfaces;

/**
 * Interface for drivetrain systems that Novad can control
 * Implement this interface to connect Novad to your drivetrain
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public interface NovadDrivetrain {

    /**
     * Drive the robot with field-centric or robot-centric controls
     * @param forward Forward/backward power (-1 to 1, positive = forward)
     * @param strafe Left/right power (-1 to 1, positive = right)
     * @param rotation Rotation power (-1 to 1, positive = counter-clockwise)
     */
    void drive(double forward, double strafe, double rotation);

    /**
     * Drive the robot with field-centric controls using heading
     * @param forward Forward power (-1 to 1)
     * @param strafe Strafe power (-1 to 1)
     * @param rotation Rotation power (-1 to 1)
     * @param heading Current robot heading in radians for field-centric
     */
    void driveFieldCentric(double forward, double strafe, double rotation, double heading);

    /**
     * Set individual motor powers for mecanum drive
     * @param frontLeft Front left motor power
     * @param frontRight Front right motor power
     * @param backLeft Back left motor power
     * @param backRight Back right motor power
     */
    void setMotorPowers(double frontLeft, double frontRight, double backLeft, double backRight);

    /**
     * Stop all motors immediately
     */
    void stop();

    /**
     * Get the maximum speed multiplier
     * @return Speed multiplier (0 to 1)
     */
    double getMaxSpeed();

    /**
     * Set the maximum speed multiplier
     * @param maxSpeed Speed multiplier (0 to 1)
     */
    void setMaxSpeed(double maxSpeed);

    /**
     * Check if the drivetrain is busy (e.g., running a trajectory)
     * @return True if busy
     */
    default boolean isBusy() {
        return false;
    }

    /**
     * Get the number of drive motors
     * @return Number of motors
     */
    default int getMotorCount() {
        return 4;
    }
}
