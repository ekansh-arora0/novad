package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

/**
 * Mecanum Drivetrain Controller
 * 
 * Handles motor control for a standard mecanum drivetrain.
 * Supports robot-centric and field-centric driving modes.
 */
public class MecanumDrivetrain {
    
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

    /**
     * Create a mecanum drivetrain
     * 
     * @param hardwareMap Hardware map from OpMode
     * @param flName Front left motor name
     * @param frName Front right motor name
     * @param blName Back left motor name
     * @param brName Back right motor name
     * @param flDir Front left motor direction
     * @param frDir Front right motor direction
     * @param blDir Back left motor direction
     * @param brDir Back right motor direction
     * @param useBrakeMode Whether to use brake mode when stopped
     */
    public MecanumDrivetrain(
            HardwareMap hardwareMap,
            String flName, String frName, String blName, String brName,
            DcMotorSimple.Direction flDir, DcMotorSimple.Direction frDir,
            DcMotorSimple.Direction blDir, DcMotorSimple.Direction brDir,
            boolean useBrakeMode) {
        
        frontLeft = hardwareMap.get(DcMotorEx.class, flName);
        frontRight = hardwareMap.get(DcMotorEx.class, frName);
        backLeft = hardwareMap.get(DcMotorEx.class, blName);
        backRight = hardwareMap.get(DcMotorEx.class, brName);
        
        frontLeft.setDirection(flDir);
        frontRight.setDirection(frDir);
        backLeft.setDirection(blDir);
        backRight.setDirection(brDir);
        
        DcMotor.ZeroPowerBehavior behavior = useBrakeMode 
            ? DcMotor.ZeroPowerBehavior.BRAKE 
            : DcMotor.ZeroPowerBehavior.FLOAT;
        
        frontLeft.setZeroPowerBehavior(behavior);
        frontRight.setZeroPowerBehavior(behavior);
        backLeft.setZeroPowerBehavior(behavior);
        backRight.setZeroPowerBehavior(behavior);
        
        // Run without encoder - we use external odometry
        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
    }

    /**
     * Drive the robot (robot-centric)
     * 
     * @param forward Forward/backward power (-1 to 1, positive = forward)
     * @param strafe Left/right power (-1 to 1, positive = right)
     * @param rotate Rotation power (-1 to 1, positive = counter-clockwise)
     */
    public void drive(double forward, double strafe, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;
        
        // Normalize if any value exceeds 1.0
        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), 
                     Math.max(Math.abs(bl), Math.abs(br))));
        if (max > 1.0) {
            fl /= max;
            fr /= max;
            bl /= max;
            br /= max;
        }
        
        frontLeft.setPower(fl);
        frontRight.setPower(fr);
        backLeft.setPower(bl);
        backRight.setPower(br);
    }

    /**
     * Drive the robot (field-centric)
     * 
     * @param forward Forward power in field coordinates
     * @param strafe Strafe power in field coordinates
     * @param rotate Rotation power
     * @param robotHeading Current robot heading in radians
     */
    public void driveFieldCentric(double forward, double strafe, double rotate, double robotHeading) {
        // Rotate input vector by -heading to convert field-centric to robot-centric
        double cos = Math.cos(-robotHeading);
        double sin = Math.sin(-robotHeading);
        
        double robotForward = forward * cos - strafe * sin;
        double robotStrafe = forward * sin + strafe * cos;
        
        drive(robotForward, robotStrafe, rotate);
    }

    /**
     * Stop all motors
     */
    public void stop() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }
}
