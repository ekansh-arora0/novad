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
    private double lastCommandedLF = 0, lastCommandedRF = 0, lastCommandedLB = 0, lastCommandedRB = 0;

    private static final double MAX_VELOCITY = 60.0;
    private static final double MAX_ANGULAR_VEL = Math.PI;
    private static final double POSITION_DEADZONE = 0.02;
    private static final double HEADING_DEADZONE = 0.005;

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

        pinpoint.update();
        Pose2D pose = pinpoint.getPosition();
        double currentX = pose.getX(DistanceUnit.INCH);
        double currentY = pose.getY(DistanceUnit.INCH);
        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);

        long now = System.nanoTime();
        double dt = (now - lastTimeNanos) * 1e-9;
        lastTimeNanos = now;
        if (dt < 0.001) dt = 0.001;

        double actualDX = currentX - lastX;
        double actualDY = currentY - lastY;
        double actualDH = normalizeAngle(currentHeading - lastHeading);

        double expStrafe = (lastCommandedLF - lastCommandedRF - lastCommandedLB + lastCommandedRB) / 4.0;
        double expForward = (lastCommandedLF + lastCommandedRF + lastCommandedLB + lastCommandedRB) / 4.0;
        double expRotate = (-lastCommandedLF + lastCommandedRF - lastCommandedLB + lastCommandedRB) / 4.0;

        double expDX = expStrafe * MAX_VELOCITY * dt;
        double expDY = expForward * MAX_VELOCITY * dt;
        double expDH = expRotate * MAX_ANGULAR_VEL * dt;

        double discX = actualDX - expDX;
        double discY = actualDY - expDY;
        double discH = normalizeAngle(actualDH - expDH);

        if (Math.abs(discX) < POSITION_DEADZONE) discX = 0;
        if (Math.abs(discY) < POSITION_DEADZONE) discY = 0;
        if (Math.abs(discH) < HEADING_DEADZONE) discH = 0;

        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

        double corrX = clamp(-xController.calculate(discX), -maxPower, maxPower);
        double corrY = clamp(-yController.calculate(discY), -maxPower, maxPower);
        double corrH = clamp(-headingController.calculate(discH), -maxPower, maxPower);

        double corrLF = corrY + corrX + corrH;
        double corrRF = corrY - corrX - corrH;
        double corrLB = corrY - corrX + corrH;
        double corrRB = corrY + corrX - corrH;

        double finalLF = clamp(intendedLF + corrLF, -1.0, 1.0);
        double finalRF = clamp(intendedRF + corrRF, -1.0, 1.0);
        double finalLB = clamp(intendedLB + corrLB, -1.0, 1.0);
        double finalRB = clamp(intendedRB + corrRB, -1.0, 1.0);

        frontLeft.setPower(finalLF);
        frontRight.setPower(finalRF);
        backLeft.setPower(finalLB);
        backRight.setPower(finalRB);

        lastCommandedLF = finalLF;
        lastCommandedRF = finalRF;
        lastCommandedLB = finalLB;
        lastCommandedRB = finalRB;

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
        lastCommandedLF = lastCommandedRF = lastCommandedLB = lastCommandedRB = 0;
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

        double corrX = clamp(xController.calculate(-errorX), -1.0, 1.0);
        double corrY = clamp(yController.calculate(-errorY), -1.0, 1.0);
        double corrH = clamp(headingController.calculate(-errorH), -1.0, 1.0);

        double lf = corrY + corrX + corrH;
        double rf = corrY - corrX - corrH;
        double lb = corrY - corrX + corrH;
        double rb = corrY + corrX - corrH;

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
