package org.firstinspires.ftc.teamcode.novad;package org.firstinspires.ftc.teamcode.novad;



import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.util.ElapsedTime;

import com.qualcomm.robotcore.hardware.DcMotorEx;import org.firstinspires.ftc.teamcode.NovadSetup;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.util.ElapsedTime;/**

 * NOVAD - FTC Defense Library

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; * 

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; * API:

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D; *   novad.defense(strafe, forward, rotate);  // Drive with push resistance

 *   novad.lockdown();                         // Immovable mode

/** *   novad.unlock();                           // Exit lockdown

 * NOVAD - FTC Defense Library */

 * public class Novad {

 * Usage:

 *   Novad novad = Constants.createNovad(hardwareMap);    public static final String VERSION = "2.0.0";

 *   

 *   // In loop:    private final Localizer localizer;

 *   novad.defense(strafe, forward, rotate);  // Drive with push resistance    private final MecanumDrivetrain drivetrain;

 *   novad.lockdown();                         // Become immovable

 *   novad.unlock();                           // Exit lockdown    private final PIDController xController;

 */    private final PIDController yController;

public class Novad {    private final PIDController headingController;



    public static final String VERSION = "2.0.0";    private Pose2d lastPose;

    private Pose2d lockdownTarget = null;

    // Hardware    

    private final GoBildaPinpointDriver pinpoint;    private final ElapsedTime loopTimer = new ElapsedTime();

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;    private double lastLoopTime = 0;



    // Controllers    private boolean inLockdown = false;

    private final PIDController xController;

    private final PIDController yController;    private static final double MAX_VELOCITY = 40.0;

    private final PIDController headingController;    private static final double MAX_ANGULAR_VEL = Math.PI;



    // Settings    public static Novad create(HardwareMap hardwareMap) {

    private final DefenseSettings settings;        Localizer localizer;

    private final PIDFCoefficients translationalPIDF;

    private final PIDFCoefficients headingPIDF;        if (NovadSetup.USE_PINPOINT) {

            localizer = new PinpointLocalizer(hardwareMap, NovadSetup.PINPOINT_NAME);

    // State        } else if (NovadSetup.USE_THREE_WHEEL) {

    private Pose2d lastPose;            localizer = new ThreeWheelLocalizer(

    private Pose2d lockdownTarget = null;                hardwareMap,

    private boolean inLockdown = false;                NovadSetup.LEFT_ENCODER,

    private double headingOffset = 0;                NovadSetup.RIGHT_ENCODER,

                    NovadSetup.CENTER_ENCODER,

    private final ElapsedTime loopTimer = new ElapsedTime();                NovadSetup.TICKS_TO_INCHES,

    private double lastLoopTime = 0;                NovadSetup.TRACK_WIDTH,

                NovadSetup.CENTER_OFFSET,

    // Constants                NovadSetup.LEFT_ENCODER_REVERSED,

    private static final double MAX_VELOCITY = 40.0;  // inches per second                NovadSetup.RIGHT_ENCODER_REVERSED,

    private static final double MAX_ANGULAR_VEL = Math.PI;  // radians per second                NovadSetup.CENTER_ENCODER_REVERSED

            );

    /**        } else {

     * Create Novad using builder (preferred method)            throw new IllegalStateException("Enable USE_PINPOINT or USE_THREE_WHEEL in NovadSetup");

     */        }

    Novad(HardwareMap hardwareMap,

          MecanumConstants driveConstants,        return new Novad(localizer, new MecanumDrivetrain(hardwareMap));

          PinpointConstants localizerConstants,    }

          PIDFCoefficients translationalPIDF,

          PIDFCoefficients headingPIDF,    public Novad(Localizer localizer, MecanumDrivetrain drivetrain) {

          DefenseSettings settings) {        this.localizer = localizer;

                this.drivetrain = drivetrain;

        this.translationalPIDF = translationalPIDF;

        this.headingPIDF = headingPIDF;        this.xController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

        this.settings = settings;        this.yController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

        this.headingController = new PIDController(NovadSetup.HEADING_P, NovadSetup.HEADING_I, NovadSetup.HEADING_D, NovadSetup.HEADING_F);

        // Initialize Pinpoint

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizerConstants.getHardwareMapName());        localizer.update();

                lastPose = localizer.getPose();

        if (localizerConstants.isUsingCustomResolution()) {        loopTimer.reset();

            pinpoint.setEncoderResolution(localizerConstants.getCustomEncoderResolution());    }

        } else {

            pinpoint.setEncoderResolution(localizerConstants.getEncoderResolution());    /**

        }     * DEFENSE MODE - Call in your loop.

             * Drives robot while resisting external pushes.

        pinpoint.setEncoderDirections(     */

            localizerConstants.getForwardEncoderDirection(),    public void defense(double strafe, double forward, double rotate) {

            localizerConstants.getStrafeEncoderDirection()        if (inLockdown) {

        );            holdPosition();

                    return;

        pinpoint.resetPosAndIMU();        }



        // Initialize motors        localizer.update();

        frontLeft = hardwareMap.get(DcMotorEx.class, driveConstants.getLeftFrontMotorName());        Pose2d currentPose = localizer.getPose();

        frontRight = hardwareMap.get(DcMotorEx.class, driveConstants.getRightFrontMotorName());        

        backLeft = hardwareMap.get(DcMotorEx.class, driveConstants.getLeftRearMotorName());        double dt = getDeltaTime();

        backRight = hardwareMap.get(DcMotorEx.class, driveConstants.getRightRearMotorName());        updatePIDGains();



        frontLeft.setDirection(driveConstants.getLeftFrontMotorDirection());        double actualDX = currentPose.x - lastPose.x;

        frontRight.setDirection(driveConstants.getRightFrontMotorDirection());        double actualDY = currentPose.y - lastPose.y;

        backLeft.setDirection(driveConstants.getLeftRearMotorDirection());        double actualDH = normalizeAngle(currentPose.heading - lastPose.heading);

        backRight.setDirection(driveConstants.getRightRearMotorDirection());

        double expectedVX = strafe * MAX_VELOCITY;

        DcMotor.ZeroPowerBehavior behavior = driveConstants.getUseBrakeMode()        double expectedVY = forward * MAX_VELOCITY;

            ? DcMotor.ZeroPowerBehavior.BRAKE        double expectedVH = rotate * MAX_ANGULAR_VEL;

            : DcMotor.ZeroPowerBehavior.FLOAT;

        if (NovadSetup.USE_FIELD_CENTRIC) {

        frontLeft.setZeroPowerBehavior(behavior);            double cos = Math.cos(currentPose.heading);

        frontRight.setZeroPowerBehavior(behavior);            double sin = Math.sin(currentPose.heading);

        backLeft.setZeroPowerBehavior(behavior);            double temp = expectedVX;

        backRight.setZeroPowerBehavior(behavior);            expectedVX = temp * cos - expectedVY * sin;

            expectedVY = temp * sin + expectedVY * cos;

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        }

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        double expectedDX = expectedVX * dt;

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        double expectedDY = expectedVY * dt;

        double expectedDH = expectedVH * dt;

        // Initialize PID controllers

        xController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        double discX = actualDX - expectedDX;

        yController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        double discY = actualDY - expectedDY;

        headingController = new PIDController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);        double discH = normalizeAngle(actualDH - expectedDH);



        // Initialize pose        if (Math.abs(discX) < 0.05) discX = 0;

        pinpoint.update();        if (Math.abs(discY) < 0.05) discY = 0;

        lastPose = getPoseFromPinpoint();        if (Math.abs(discH) < 0.01) discH = 0;

        loopTimer.reset();

    }        double corrX = clamp(-xController.calculate(discX), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

        double corrY = clamp(-yController.calculate(discY), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

    /**        double corrH = clamp(-headingController.calculate(discH), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

     * DEFENSE MODE - Drive while resisting pushes

     */        double finalStrafe = strafe + corrX;

    public void defense(double strafe, double forward, double rotate) {        double finalForward = forward + corrY;

        if (inLockdown) {        double finalRotate = rotate + corrH;

            holdPosition();

            return;        double max = Math.max(1.0, Math.abs(finalStrafe) + Math.abs(finalForward) + Math.abs(finalRotate));

        }        finalStrafe /= max;

        finalForward /= max;

        pinpoint.update();        finalRotate /= max;

        Pose2d currentPose = getPoseFromPinpoint();

                if (NovadSetup.USE_FIELD_CENTRIC) {

        double dt = getDeltaTime();            drivetrain.driveFieldCentric(finalStrafe, finalForward, finalRotate, currentPose.heading);

        updatePIDGains();        } else {

            drivetrain.drive(finalStrafe, finalForward, finalRotate);

        // What actually happened        }

        double actualDX = currentPose.x - lastPose.x;

        double actualDY = currentPose.y - lastPose.y;        lastPose = currentPose;

        double actualDH = normalizeAngle(currentPose.heading - lastPose.heading);    }



        // What we expected based on joystick    /**

        double expectedVX = strafe * MAX_VELOCITY;     * LOCKDOWN - Robot becomes immovable.

        double expectedVY = forward * MAX_VELOCITY;     */

        double expectedVH = rotate * MAX_ANGULAR_VEL;    public void lockdown() {

        if (!inLockdown) {

        if (settings.getUseFieldCentric()) {            localizer.update();

            double cos = Math.cos(currentPose.heading);            lockdownTarget = localizer.getPose();

            double sin = Math.sin(currentPose.heading);            inLockdown = true;

            double temp = expectedVX;            xController.reset();

            expectedVX = temp * cos - expectedVY * sin;            yController.reset();

            expectedVY = temp * sin + expectedVY * cos;            headingController.reset();

        }        }

        holdPosition();

        double expectedDX = expectedVX * dt;    }

        double expectedDY = expectedVY * dt;

        double expectedDH = expectedVH * dt;    /**

     * Exit lockdown.

        // Discrepancy = external push     */

        double discX = actualDX - expectedDX;    public void unlock() {

        double discY = actualDY - expectedDY;        inLockdown = false;

        double discH = normalizeAngle(actualDH - expectedDH);        localizer.update();

        lastPose = localizer.getPose();

        // Deadzone    }

        if (Math.abs(discX) < 0.05) discX = 0;

        if (Math.abs(discY) < 0.05) discY = 0;    public boolean isLocked() {

        if (Math.abs(discH) < 0.01) discH = 0;        return inLockdown;

    }

        // Calculate corrections

        double maxPower = settings.getMaxCorrectionPower();    public Pose2d getPose() {

        double corrX = clamp(-xController.calculate(discX), -maxPower, maxPower);        return localizer.getPose();

        double corrY = clamp(-yController.calculate(discY), -maxPower, maxPower);    }

        double corrH = clamp(-headingController.calculate(discH), -maxPower, maxPower);

    public void resetHeading() {

        // Apply driver input + correction        localizer.update();

        double finalStrafe = strafe + corrX;        Pose2d p = localizer.getPose();

        double finalForward = forward + corrY;        lastPose = new Pose2d(p.x, p.y, 0);

        double finalRotate = rotate + corrH;        if (inLockdown && lockdownTarget != null) {

            lockdownTarget = new Pose2d(lockdownTarget.x, lockdownTarget.y, 0);

        // Normalize        }

        double max = Math.max(1.0, Math.abs(finalStrafe) + Math.abs(finalForward) + Math.abs(finalRotate));    }

        finalStrafe /= max;

        finalForward /= max;    private void holdPosition() {

        finalRotate /= max;        localizer.update();

        Pose2d current = localizer.getPose();

        // Drive        updatePIDGains();

        if (settings.getUseFieldCentric()) {

            driveFieldCentric(finalStrafe, finalForward, finalRotate, currentPose.heading);        double errX = lockdownTarget.x - current.x;

        } else {        double errY = lockdownTarget.y - current.y;

            drive(finalStrafe, finalForward, finalRotate);        double errH = normalizeAngle(lockdownTarget.heading - current.heading);

        }

        double corrX = clamp(xController.calculate(errX), -1.0, 1.0);

        lastPose = currentPose;        double corrY = clamp(yController.calculate(errY), -1.0, 1.0);

    }        double corrH = clamp(headingController.calculate(errH), -1.0, 1.0);



    /**        if (NovadSetup.USE_FIELD_CENTRIC) {

     * LOCKDOWN - Become completely immovable            drivetrain.driveFieldCentric(corrX, corrY, corrH, current.heading);

     */        } else {

    public void lockdown() {            drivetrain.drive(corrX, corrY, corrH);

        if (!inLockdown) {        }

            pinpoint.update();    }

            lockdownTarget = getPoseFromPinpoint();

            inLockdown = true;    private double getDeltaTime() {

            xController.reset();        double now = loopTimer.seconds();

            yController.reset();        double dt = now - lastLoopTime;

            headingController.reset();        lastLoopTime = now;

        }        return (dt <= 0 || dt > 0.5) ? 0.02 : dt;

    }    }



    /**    private void updatePIDGains() {

     * UNLOCK - Exit lockdown, return to defense mode        xController.setGains(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

     */        yController.setGains(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

    public void unlock() {        headingController.setGains(NovadSetup.HEADING_P, NovadSetup.HEADING_I, NovadSetup.HEADING_D, NovadSetup.HEADING_F);

        if (inLockdown) {    }

            inLockdown = false;

            lockdownTarget = null;    private double normalizeAngle(double a) {

            pinpoint.update();        while (a > Math.PI) a -= 2 * Math.PI;

            lastPose = getPoseFromPinpoint();        while (a < -Math.PI) a += 2 * Math.PI;

        }        return a;

    }    }



    /**    private double clamp(double v, double min, double max) {

     * Check if currently in lockdown mode        return Math.max(min, Math.min(max, v));

     */    }

    public boolean isLocked() {}

        return inLockdown;
    }

    /**
     * Get current robot pose
     */
    public Pose2d getPose() {
        pinpoint.update();
        return getPoseFromPinpoint();
    }

    /**
     * Reset heading to zero (for field-centric)
     */
    public void resetHeading() {
        headingOffset = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);
    }

    // ═══════════════════════════════════════════════════════════════════════════
    //                           PRIVATE METHODS
    // ═══════════════════════════════════════════════════════════════════════════

    private void holdPosition() {
        if (lockdownTarget == null) return;

        pinpoint.update();
        Pose2d currentPose = getPoseFromPinpoint();
        updatePIDGains();

        double errorX = lockdownTarget.x - currentPose.x;
        double errorY = lockdownTarget.y - currentPose.y;
        double errorH = normalizeAngle(lockdownTarget.heading - currentPose.heading);

        double maxPower = settings.getMaxCorrectionPower();
        double corrX = clamp(xController.calculate(errorX), -maxPower, maxPower);
        double corrY = clamp(yController.calculate(errorY), -maxPower, maxPower);
        double corrH = clamp(headingController.calculate(errorH), -maxPower, maxPower);

        drive(corrX, corrY, corrH);
    }

    private Pose2d getPoseFromPinpoint() {
        Pose2D pose = pinpoint.getPosition();
        return new Pose2d(
            pose.getX(DistanceUnit.INCH),
            pose.getY(DistanceUnit.INCH),
            normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset)
        );
    }

    private void drive(double strafe, double forward, double rotate) {
        double fl = forward + strafe + rotate;
        double fr = forward - strafe - rotate;
        double bl = forward - strafe + rotate;
        double br = forward + strafe - rotate;

        double max = Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br))));
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

    private void driveFieldCentric(double strafe, double forward, double rotate, double heading) {
        double cos = Math.cos(-heading);
        double sin = Math.sin(-heading);
        double rotatedStrafe = strafe * cos - forward * sin;
        double rotatedForward = strafe * sin + forward * cos;
        drive(rotatedStrafe, rotatedForward, rotate);
    }

    private double getDeltaTime() {
        double currentTime = loopTimer.seconds();
        double dt = currentTime - lastLoopTime;
        lastLoopTime = currentTime;
        return Math.max(dt, 0.001);
    }

    private void updatePIDGains() {
        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);
        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);
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
