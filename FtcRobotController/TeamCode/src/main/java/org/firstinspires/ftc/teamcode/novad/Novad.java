package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Gamepad;
import com.qualcomm.robotcore.hardware.HardwareMap;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

public class Novad {
    public static final String VERSION = "2.2.0";

    private final GoBildaPinpointDriver pinpoint;
    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;
    private final PIDController xController, yController, headingController;
    private final double maxPower;
    private final boolean useFieldCentric;
    private PIDFCoefficients translationalPIDF, headingPIDF;

    private double lastX, lastY, lastHeading;
    private double lockX, lockY, lockHeading;
    private boolean inLockdown = false;
    private double headingOffset = 0;
    private long lastTimeNanos;
    
    // Track intended motor powers BEFORE correction (what driver commanded)
    private double intendedLF = 0, intendedRF = 0, intendedLB = 0, intendedRB = 0;
    
    // Track expected position based on intended commands
    private double expectedX, expectedY, expectedHeading;

    private static final double MAX_VELOCITY = 60.0;  // inches per second at full power
    private static final double MAX_ANGULAR_VEL = Math.PI;  // radians per second at full power
    private static final double POSITION_DEADZONE = 0.15;  // ignore small discrepancies (inches)
    private static final double HEADING_DEADZONE = 0.02;   // ignore small heading discrepancies (radians)

    Novad(HardwareMap hardwareMap, MecanumConstants drive, PinpointConstants localizer,
          PIDFCoefficients translationalPIDF, PIDFCoefficients headingPIDF, DefenseSettings settings) {
        this.translationalPIDF = translationalPIDF;
        this.headingPIDF = headingPIDF;
        this.maxPower = settings.getMaxCorrectionPower();
        this.useFieldCentric = settings.getUseFieldCentric();

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizer.getHardwareMapName());
        if (localizer.isUsingCustomResolution()) {
            pinpoint.setEncoderResolution(localizer.getCustomEncoderResolution(), DistanceUnit.MM);
        } else {
            pinpoint.setEncoderResolution(localizer.getEncoderResolution());
        }
        pinpoint.setEncoderDirections(localizer.getForwardEncoderDirection(), localizer.getStrafeEncoderDirection());
        pinpoint.resetPosAndIMU();

        frontLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftFrontMotorName());
        frontRight = hardwareMap.get(DcMotorEx.class, drive.getRightFrontMotorName());
        backLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftRearMotorName());
        backRight = hardwareMap.get(DcMotorEx.class, drive.getRightRearMotorName());

        frontLeft.setDirection(drive.getLeftFrontMotorDirection());
        frontRight.setDirection(drive.getRightFrontMotorDirection());
        backLeft.setDirection(drive.getLeftRearMotorDirection());
        backRight.setDirection(drive.getRightRearMotorDirection());

        DcMotor.ZeroPowerBehavior brake = drive.getUseBrakeMode() ? DcMotor.ZeroPowerBehavior.BRAKE : DcMotor.ZeroPowerBehavior.FLOAT;
        frontLeft.setZeroPowerBehavior(brake);
        frontRight.setZeroPowerBehavior(brake);
        backLeft.setZeroPowerBehavior(brake);
        backRight.setZeroPowerBehavior(brake);

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        xController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        yController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        headingController = new PIDController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        lastX = pose.getX(DistanceUnit.INCH);
        lastY = pose.getY(DistanceUnit.INCH);
        lastHeading = pose.getHeading(AngleUnit.RADIANS);
        expectedX = lastX;
        expectedY = lastY;
        expectedHeading = lastHeading;
        lastTimeNanos = System.nanoTime();
    }

    public void defense(Gamepad gamepad) {
        if (inLockdown) { holdPosition(); return; }
        double strafe = gamepad.left_stick_x;
        double forward = -gamepad.left_stick_y;
        double rotate = gamepad.right_stick_x;
        applyDefense(strafe, forward, rotate);
    }

    public void defense(double strafe, double forward, double rotate) {
        if (inLockdown) { holdPosition(); return; }
        applyDefense(strafe, forward, rotate);
    }

    private void applyDefense(double strafe, double forward, double rotate) {
        double heading = lastHeading;
        if (useFieldCentric) {
            double cos = Math.cos(heading);
            double sin = Math.sin(heading);
            double temp = strafe;
            strafe = temp * cos - forward * sin;
            forward = temp * sin + forward * cos;
        }
        double lf = forward + strafe + rotate;
        double rf = forward - strafe - rotate;
        double lb = forward - strafe + rotate;
        double rb = forward + strafe - rotate;
        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb))));
        if (max > 1.0) { lf /= max; rf /= max; lb /= max; rb /= max; }
        defenseWithMotors(lf, rf, lb, rb);
    }

    public void defenseWithMotors(double intendedLF, double intendedRF, double intendedLB, double intendedRB) {
        if (inLockdown) { holdPosition(); return; }

        // Store intended powers (what the driver wants, BEFORE correction)
        this.intendedLF = intendedLF;
        this.intendedRF = intendedRF;
        this.intendedLB = intendedLB;
        this.intendedRB = intendedRB;

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double currentX = pose.getX(DistanceUnit.INCH);
        double currentY = pose.getY(DistanceUnit.INCH);
        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);

        long now = System.nanoTime();
        double dt = (now - lastTimeNanos) * 1e-9;
        lastTimeNanos = now;
        if (dt < 0.001) dt = 0.001;
        if (dt > 0.1) dt = 0.1;  // Cap dt to prevent huge jumps

        // Calculate expected movement from INTENDED commands (not corrected ones!)
        // This is robot-relative movement
        // Inverse kinematics: motor powers → strafe/forward/rotate
        // Given: LF = F+S+R, RF = F-S-R, LB = F-S+R, RB = F+S-R
        double robotForward = (intendedLF + intendedRF + intendedLB + intendedRB) / 4.0;
        double robotStrafe = (intendedLF - intendedRF - intendedLB + intendedRB) / 4.0;
        double robotRotate = (intendedLF - intendedRF + intendedLB - intendedRB) / 4.0;

        // Convert robot-relative to field-relative using current heading
        // Robot X (strafe) = right, Robot Y (forward) = front
        // Field X = right on field, Field Y = forward on field
        // Rotation matrix: [cos -sin; sin cos] for robot->field
        double cos = Math.cos(currentHeading);
        double sin = Math.sin(currentHeading);
        double fieldDX = (robotForward * sin + robotStrafe * cos) * MAX_VELOCITY * dt;
        double fieldDY = (robotForward * cos - robotStrafe * sin) * MAX_VELOCITY * dt;
        double fieldDH = robotRotate * MAX_ANGULAR_VEL * dt;

        // Update expected position based on driver intent
        expectedX += fieldDX;
        expectedY += fieldDY;
        expectedHeading = normalizeAngle(expectedHeading + fieldDH);

        // Calculate error: where we should be vs where we are
        double errorX = expectedX - currentX;
        double errorY = expectedY - currentY;
        double errorH = normalizeAngle(expectedHeading - currentHeading);

        // Apply deadzone - don't correct tiny discrepancies (prevents jitter!)
        if (Math.abs(errorX) < POSITION_DEADZONE) { errorX = 0; expectedX = currentX; }
        if (Math.abs(errorY) < POSITION_DEADZONE) { errorY = 0; expectedY = currentY; }
        if (Math.abs(errorH) < HEADING_DEADZONE) { errorH = 0; expectedHeading = currentHeading; }

        // Update PIDF coefficients
        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

        // Calculate correction in field frame
        // Error is positive when we're behind where we should be, so correction should be positive
        double corrFieldX = clamp(xController.calculate(errorX), -maxPower, maxPower);
        double corrFieldY = clamp(yController.calculate(errorY), -maxPower, maxPower);
        double corrH = clamp(headingController.calculate(errorH), -maxPower, maxPower);

        // Convert field-relative correction back to robot-relative
        // Inverse rotation: [cos sin; -sin cos] for field->robot
        double corrRobotStrafe = corrFieldX * cos - corrFieldY * sin;
        double corrRobotForward = corrFieldX * sin + corrFieldY * cos;

        // Apply mecanum kinematics for correction
        double corrLF = corrRobotForward + corrRobotStrafe + corrH;
        double corrRF = corrRobotForward - corrRobotStrafe - corrH;
        double corrLB = corrRobotForward - corrRobotStrafe + corrH;
        double corrRB = corrRobotForward + corrRobotStrafe - corrH;

        // Combine intended power with correction
        double finalLF = clamp(intendedLF + corrLF, -1.0, 1.0);
        double finalRF = clamp(intendedRF + corrRF, -1.0, 1.0);
        double finalLB = clamp(intendedLB + corrLB, -1.0, 1.0);
        double finalRB = clamp(intendedRB + corrRB, -1.0, 1.0);

        // Apply to motors
        frontLeft.setPower(finalLF);
        frontRight.setPower(finalRF);
        backLeft.setPower(finalLB);
        backRight.setPower(finalRB);

        // Update tracking (position only, NOT the intended powers!)
        lastX = currentX;
        lastY = currentY;
        lastHeading = currentHeading;
    }

    public void lockdown() {
        if (!inLockdown) {
            pinpoint.update();
            Pose2D pose = pinpoint.getPosition();
            lockX = pose.getX(DistanceUnit.INCH);
            lockY = pose.getY(DistanceUnit.INCH);
            lockHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);
            xController.reset();
            yController.reset();
            headingController.reset();
            inLockdown = true;
        }
        holdPosition();
    }

    public void unlock() {
        inLockdown = false;
        intendedLF = intendedRF = intendedLB = intendedRB = 0;
        // Sync expected position to actual on unlock
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        expectedX = pose.getX(DistanceUnit.INCH);
        expectedY = pose.getY(DistanceUnit.INCH);
        expectedHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);
        xController.reset();
        yController.reset();
        headingController.reset();
    }

    public boolean isLocked() { return inLockdown; }
    public Pose2d getPose() { return new Pose2d(lastX, lastY, normalizeAngle(lastHeading)); }

    public void resetHeading() {
        pinpoint.update();
        headingOffset = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    public void resetPosition() {
        pinpoint.resetPosAndIMU();
        headingOffset = lastX = lastY = lastHeading = 0;
    }

    public void setTranslationalPIDF(PIDFCoefficients pidf) { this.translationalPIDF = pidf; }
    public void setHeadingPIDF(PIDFCoefficients pidf) { this.headingPIDF = pidf; }
    public double getLoopTimeMs() { return (System.nanoTime() - lastTimeNanos) / 1_000_000.0; }

    private void holdPosition() {
        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double currentX = pose.getX(DistanceUnit.INCH);
        double currentY = pose.getY(DistanceUnit.INCH);
        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);

        double errorX = lockX - currentX;
        double errorY = lockY - currentY;
        double errorH = normalizeAngle(lockHeading - currentHeading);

        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

        // Error is positive when we're behind lock position, correction should push us there
        double corrX = clamp(xController.calculate(errorX), -1.0, 1.0);
        double corrY = clamp(yController.calculate(errorY), -1.0, 1.0);
        double corrH = clamp(headingController.calculate(errorH), -1.0, 1.0);

        // Convert field-relative corrections to robot-relative
        // Inverse rotation: [cos sin; -sin cos] for field->robot
        double cos = Math.cos(currentHeading);
        double sin = Math.sin(currentHeading);
        double corrRobotStrafe = corrX * cos - corrY * sin;
        double corrRobotForward = corrX * sin + corrY * cos;

        double lf = corrRobotForward + corrRobotStrafe + corrH;
        double rf = corrRobotForward - corrRobotStrafe - corrH;
        double lb = corrRobotForward - corrRobotStrafe + corrH;
        double rb = corrRobotForward + corrRobotStrafe - corrH;

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb))));
        if (max > 1.0) { lf /= max; rf /= max; lb /= max; rb /= max; }

        frontLeft.setPower(lf);
        frontRight.setPower(rf);
        backLeft.setPower(lb);
        backRight.setPower(rb);

        lastX = currentX;
        lastY = currentY;
        lastHeading = currentHeading;
    }

    private static double normalizeAngle(double angle) {
        while (angle > Math.PI) angle -= 2 * Math.PI;
        while (angle < -Math.PI) angle += 2 * Math.PI;
        return angle;
    }

    private static double clamp(double value, double min, double max) {
        return Math.max(min, Math.min(max, value));
    }
}
