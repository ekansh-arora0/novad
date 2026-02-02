package com.novad.adapters;

import com.novad.interfaces.NovadDrivetrain;

/**
 * Generic Mecanum Drivetrain Adapter
 * 
 * Use this class to connect Novad to your mecanum drivetrain.
 * You need to provide the motor setters via functional interfaces.
 * 
 * Example:
 * <pre>
 * DcMotor fl = hardwareMap.get(DcMotor.class, "frontLeft");
 * DcMotor fr = hardwareMap.get(DcMotor.class, "frontRight");
 * DcMotor bl = hardwareMap.get(DcMotor.class, "backLeft");
 * DcMotor br = hardwareMap.get(DcMotor.class, "backRight");
 * 
 * MecanumDriveAdapter drivetrain = new MecanumDriveAdapter(
 *     fl::setPower, fr::setPower, bl::setPower, br::setPower
 * );
 * </pre>
 * 
 * @author Novad Defense Library
 * @version 1.0.0
 */
public class MecanumDriveAdapter implements NovadDrivetrain {

    /**
     * Functional interface for setting motor power
     */
    @FunctionalInterface
    public interface MotorPowerSetter {
        void setPower(double power);
    }

    private final MotorPowerSetter frontLeft;
    private final MotorPowerSetter frontRight;
    private final MotorPowerSetter backLeft;
    private final MotorPowerSetter backRight;

    private double maxSpeed = 1.0;

    /**
     * Create a mecanum drivetrain adapter
     * @param frontLeft Front left motor power setter
     * @param frontRight Front right motor power setter
     * @param backLeft Back left motor power setter
     * @param backRight Back right motor power setter
     */
    public MecanumDriveAdapter(
            MotorPowerSetter frontLeft,
            MotorPowerSetter frontRight,
            MotorPowerSetter backLeft,
            MotorPowerSetter backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;
    }

    @Override
    public void drive(double forward, double strafe, double rotation) {
        // Mecanum drive kinematics (robot-centric)
        double fl = forward + strafe + rotation;
        double fr = forward - strafe - rotation;
        double bl = forward - strafe + rotation;
        double br = forward + strafe - rotation;

        // Normalize to max speed
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
        if (max > maxSpeed) {
            fl = fl / max * maxSpeed;
            fr = fr / max * maxSpeed;
            bl = bl / max * maxSpeed;
            br = br / max * maxSpeed;
        }

        setMotorPowers(fl, fr, bl, br);
    }

    @Override
    public void driveFieldCentric(double forward, double strafe, double rotation, double heading) {
        // Rotate input vector by negative heading for field-centric control
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);

        double rotatedForward = forward * cos - strafe * sin;
        double rotatedStrafe = forward * sin + strafe * cos;

        drive(rotatedForward, rotatedStrafe, rotation);
    }

    @Override
    public void setMotorPowers(double fl, double fr, double bl, double br) {
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    @Override
    public void stop() {
        setMotorPowers(0, 0, 0, 0);
    }

    @Override
    public double getMaxSpeed() {
        return maxSpeed;
    }

    @Override
    public void setMaxSpeed(double maxSpeed) {
        this.maxSpeed = Math.max(0, Math.min(1, maxSpeed));
    }

    @Override
    public int getMotorCount() {
        return 4;
    }
}
