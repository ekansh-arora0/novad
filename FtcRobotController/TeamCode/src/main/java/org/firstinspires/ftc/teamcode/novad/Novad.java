package org.firstinspires.ftc.teamcode.novad;package org.firstinspires.ftc.teamcode.novad;package org.firstinspires.ftc.teamcode.novad;



import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.HardwareMap;

import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;import com.qualcomm.robotcore.hardware.DcMotorEx;import org.firstinspires.ftc.teamcode.NovadSetup;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

import com.qualcomm.robotcore.hardware.HardwareMap;

/**

 * NOVAD - FTC Defense Libraryimport com.qualcomm.robotcore.util.ElapsedTime;/**

 * 

 * Ultra-low latency defense system with <20ms reaction time. * NOVAD - FTC Defense Library

 * Uses Pinpoint odometry for precise position tracking.

 * import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; * 

 * Usage:

 *   Novad novad = Constants.createNovad(hardwareMap);import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; * API:

 *   novad.defense(strafe, forward, rotate);  // In loop

 *   novad.lockdown();                         // Become immovableimport org.firstinspires.ftc.robotcore.external.navigation.Pose2D; *   novad.defense(strafe, forward, rotate);  // Drive with push resistance

 *   novad.unlock();                           // Exit lockdown

 */ *   novad.lockdown();                         // Immovable mode

public class Novad {

/** *   novad.unlock();                           // Exit lockdown

    public static final String VERSION = "2.1.0";

 * NOVAD - FTC Defense Library */

    // Hardware - direct references for speed

    private final GoBildaPinpointDriver pinpoint; * public class Novad {

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;

 * Usage:

    // Controllers

    private final PIDController xController; *   Novad novad = Constants.createNovad(hardwareMap);    public static final String VERSION = "2.0.0";

    private final PIDController yController;

    private final PIDController headingController; *   



    // Settings - cached for speed *   // In loop:    private final Localizer localizer;

    private final double maxPower;

    private final boolean useFieldCentric; *   novad.defense(strafe, forward, rotate);  // Drive with push resistance    private final MecanumDrivetrain drivetrain;

    private PIDFCoefficients translationalPIDF;

    private PIDFCoefficients headingPIDF; *   novad.lockdown();                         // Become immovable



    // State - preallocated to avoid GC *   novad.unlock();                           // Exit lockdown    private final PIDController xController;

    private double lastX, lastY, lastHeading;

    private double lockX, lockY, lockHeading; */    private final PIDController yController;

    private boolean inLockdown = false;

    private double headingOffset = 0;public class Novad {    private final PIDController headingController;

    private long lastTimeNanos;



    // Constants

    private static final double MAX_VELOCITY = 60.0;       // inches/sec (tunable)    public static final String VERSION = "2.0.0";    private Pose2d lastPose;

    private static final double MAX_ANGULAR_VEL = Math.PI; // rad/sec

    private static final double POSITION_DEADZONE = 0.02;  // inches    private Pose2d lockdownTarget = null;

    private static final double HEADING_DEADZONE = 0.005;  // radians

    // Hardware    

    /**

     * Create Novad using NovadBuilder    private final GoBildaPinpointDriver pinpoint;    private final ElapsedTime loopTimer = new ElapsedTime();

     */

    Novad(HardwareMap hardwareMap,    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;    private double lastLoopTime = 0;

          MecanumConstants drive,

          PinpointConstants localizer,

          PIDFCoefficients translationalPIDF,

          PIDFCoefficients headingPIDF,    // Controllers    private boolean inLockdown = false;

          DefenseSettings settings) {

            private final PIDController xController;

        this.translationalPIDF = translationalPIDF;

        this.headingPIDF = headingPIDF;    private final PIDController yController;    private static final double MAX_VELOCITY = 40.0;

        this.maxPower = settings.getMaxCorrectionPower();

        this.useFieldCentric = settings.getUseFieldCentric();    private final PIDController headingController;    private static final double MAX_ANGULAR_VEL = Math.PI;



        // Initialize Pinpoint - bulk read mode for speed

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizer.getHardwareMapName());

            // Settings    public static Novad create(HardwareMap hardwareMap) {

        if (localizer.isUsingCustomResolution()) {

            pinpoint.setEncoderResolution(localizer.getCustomEncoderResolution());    private final DefenseSettings settings;        Localizer localizer;

        } else {

            pinpoint.setEncoderResolution(localizer.getEncoderResolution());    private final PIDFCoefficients translationalPIDF;

        }

            private final PIDFCoefficients headingPIDF;        if (NovadSetup.USE_PINPOINT) {

        pinpoint.setEncoderDirections(

            localizer.getForwardEncoderDirection(),            localizer = new PinpointLocalizer(hardwareMap, NovadSetup.PINPOINT_NAME);

            localizer.getStrafeEncoderDirection()

        );    // State        } else if (NovadSetup.USE_THREE_WHEEL) {

        pinpoint.resetPosAndIMU();

    private Pose2d lastPose;            localizer = new ThreeWheelLocalizer(

        // Initialize motors - direct access for minimal latency

        frontLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftFrontMotorName());    private Pose2d lockdownTarget = null;                hardwareMap,

        frontRight = hardwareMap.get(DcMotorEx.class, drive.getRightFrontMotorName());

        backLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftRearMotorName());    private boolean inLockdown = false;                NovadSetup.LEFT_ENCODER,

        backRight = hardwareMap.get(DcMotorEx.class, drive.getRightRearMotorName());

    private double headingOffset = 0;                NovadSetup.RIGHT_ENCODER,

        frontLeft.setDirection(drive.getLeftFrontMotorDirection());

        frontRight.setDirection(drive.getRightFrontMotorDirection());                    NovadSetup.CENTER_ENCODER,

        backLeft.setDirection(drive.getLeftRearMotorDirection());

        backRight.setDirection(drive.getRightRearMotorDirection());    private final ElapsedTime loopTimer = new ElapsedTime();                NovadSetup.TICKS_TO_INCHES,



        DcMotor.ZeroPowerBehavior brake = drive.getUseBrakeMode()    private double lastLoopTime = 0;                NovadSetup.TRACK_WIDTH,

            ? DcMotor.ZeroPowerBehavior.BRAKE

            : DcMotor.ZeroPowerBehavior.FLOAT;                NovadSetup.CENTER_OFFSET,



        frontLeft.setZeroPowerBehavior(brake);    // Constants                NovadSetup.LEFT_ENCODER_REVERSED,

        frontRight.setZeroPowerBehavior(brake);

        backLeft.setZeroPowerBehavior(brake);    private static final double MAX_VELOCITY = 40.0;  // inches per second                NovadSetup.RIGHT_ENCODER_REVERSED,

        backRight.setZeroPowerBehavior(brake);

    private static final double MAX_ANGULAR_VEL = Math.PI;  // radians per second                NovadSetup.CENTER_ENCODER_REVERSED

        // RUN_WITHOUT_ENCODER for lowest latency

        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            );

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    /**        } else {

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

     * Create Novad using builder (preferred method)            throw new IllegalStateException("Enable USE_PINPOINT or USE_THREE_WHEEL in NovadSetup");

        // Initialize PID controllers

        xController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);     */        }

        yController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        headingController = new PIDController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);    Novad(HardwareMap hardwareMap,



        // Initialize state          MecanumConstants driveConstants,        return new Novad(localizer, new MecanumDrivetrain(hardwareMap));

        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();          PinpointConstants localizerConstants,    }

        lastX = pose.getX(DistanceUnit.INCH);

        lastY = pose.getY(DistanceUnit.INCH);          PIDFCoefficients translationalPIDF,

        lastHeading = pose.getHeading(AngleUnit.RADIANS);

        lastTimeNanos = System.nanoTime();          PIDFCoefficients headingPIDF,    public Novad(Localizer localizer, MecanumDrivetrain drivetrain) {

    }

          DefenseSettings settings) {        this.localizer = localizer;

    /**

     * DEFENSE MODE - Drive while resisting pushes                this.drivetrain = drivetrain;

     * Call this every loop iteration (~20ms or faster)

     */        this.translationalPIDF = translationalPIDF;

    public void defense(double strafe, double forward, double rotate) {

        if (inLockdown) {        this.headingPIDF = headingPIDF;        this.xController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

            holdPosition();

            return;        this.settings = settings;        this.yController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

        }

        this.headingController = new PIDController(NovadSetup.HEADING_P, NovadSetup.HEADING_I, NovadSetup.HEADING_D, NovadSetup.HEADING_F);

        // Get current pose - single I2C read

        pinpoint.update();        // Initialize Pinpoint

        Pose2D pose = pinpoint.getPosition();

        double currentX = pose.getX(DistanceUnit.INCH);        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizerConstants.getHardwareMapName());        localizer.update();

        double currentY = pose.getY(DistanceUnit.INCH);

        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);                lastPose = localizer.getPose();



        // Calculate delta time        if (localizerConstants.isUsingCustomResolution()) {        loopTimer.reset();

        long now = System.nanoTime();

        double dt = (now - lastTimeNanos) * 1e-9;            pinpoint.setEncoderResolution(localizerConstants.getCustomEncoderResolution());    }

        lastTimeNanos = now;

        if (dt < 0.001) dt = 0.001;        } else {



        // Actual movement this frame            pinpoint.setEncoderResolution(localizerConstants.getEncoderResolution());    /**

        double actualDX = currentX - lastX;

        double actualDY = currentY - lastY;        }     * DEFENSE MODE - Call in your loop.

        double actualDH = normalizeAngle(currentHeading - lastHeading);

             * Drives robot while resisting external pushes.

        // Expected movement based on joystick

        double expVX = strafe * MAX_VELOCITY;        pinpoint.setEncoderDirections(     */

        double expVY = forward * MAX_VELOCITY;

        double expVH = rotate * MAX_ANGULAR_VEL;            localizerConstants.getForwardEncoderDirection(),    public void defense(double strafe, double forward, double rotate) {



        // Field-centric transform if enabled            localizerConstants.getStrafeEncoderDirection()        if (inLockdown) {

        if (useFieldCentric) {

            double cos = Math.cos(currentHeading);        );            holdPosition();

            double sin = Math.sin(currentHeading);

            double temp = expVX;                    return;

            expVX = temp * cos - expVY * sin;

            expVY = temp * sin + expVY * cos;        pinpoint.resetPosAndIMU();        }

        }



        double expDX = expVX * dt;

        double expDY = expVY * dt;        // Initialize motors        localizer.update();

        double expDH = expVH * dt;

        frontLeft = hardwareMap.get(DcMotorEx.class, driveConstants.getLeftFrontMotorName());        Pose2d currentPose = localizer.getPose();

        // Discrepancy = external force

        double discX = actualDX - expDX;        frontRight = hardwareMap.get(DcMotorEx.class, driveConstants.getRightFrontMotorName());        

        double discY = actualDY - expDY;

        double discH = normalizeAngle(actualDH - expDH);        backLeft = hardwareMap.get(DcMotorEx.class, driveConstants.getLeftRearMotorName());        double dt = getDeltaTime();



        // Deadzone to prevent jitter        backRight = hardwareMap.get(DcMotorEx.class, driveConstants.getRightRearMotorName());        updatePIDGains();

        if (Math.abs(discX) < POSITION_DEADZONE) discX = 0;

        if (Math.abs(discY) < POSITION_DEADZONE) discY = 0;

        if (Math.abs(discH) < HEADING_DEADZONE) discH = 0;

        frontLeft.setDirection(driveConstants.getLeftFrontMotorDirection());        double actualDX = currentPose.x - lastPose.x;

        // Update PID gains (for live Dashboard tuning)

        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        frontRight.setDirection(driveConstants.getRightFrontMotorDirection());        double actualDY = currentPose.y - lastPose.y;

        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);        backLeft.setDirection(driveConstants.getLeftRearMotorDirection());        double actualDH = normalizeAngle(currentPose.heading - lastPose.heading);



        // Calculate corrections        backRight.setDirection(driveConstants.getRightRearMotorDirection());

        double corrX = clamp(-xController.calculate(discX), -maxPower, maxPower);

        double corrY = clamp(-yController.calculate(discY), -maxPower, maxPower);        double expectedVX = strafe * MAX_VELOCITY;

        double corrH = clamp(-headingController.calculate(discH), -maxPower, maxPower);

        DcMotor.ZeroPowerBehavior behavior = driveConstants.getUseBrakeMode()        double expectedVY = forward * MAX_VELOCITY;

        // Combine driver input + correction

        double finalStrafe = strafe + corrX;            ? DcMotor.ZeroPowerBehavior.BRAKE        double expectedVH = rotate * MAX_ANGULAR_VEL;

        double finalForward = forward + corrY;

        double finalRotate = rotate + corrH;            : DcMotor.ZeroPowerBehavior.FLOAT;



        // Normalize and drive        if (NovadSetup.USE_FIELD_CENTRIC) {

        driveMotors(finalStrafe, finalForward, finalRotate, currentHeading);

        frontLeft.setZeroPowerBehavior(behavior);            double cos = Math.cos(currentPose.heading);

        // Update state

        lastX = currentX;        frontRight.setZeroPowerBehavior(behavior);            double sin = Math.sin(currentPose.heading);

        lastY = currentY;

        lastHeading = currentHeading;        backLeft.setZeroPowerBehavior(behavior);            double temp = expectedVX;

    }

        backRight.setZeroPowerBehavior(behavior);            expectedVX = temp * cos - expectedVY * sin;

    /**

     * LOCKDOWN - Become completely immovable            expectedVY = temp * sin + expectedVY * cos;

     */

    public void lockdown() {        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        }

        if (!inLockdown) {

            pinpoint.update();        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            Pose2D pose = pinpoint.getPosition();

            lockX = pose.getX(DistanceUnit.INCH);        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        double expectedDX = expectedVX * dt;

            lockY = pose.getY(DistanceUnit.INCH);

            lockHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        double expectedDY = expectedVY * dt;

            inLockdown = true;

            xController.reset();        double expectedDH = expectedVH * dt;

            yController.reset();

            headingController.reset();        // Initialize PID controllers

        }

    }        xController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        double discX = actualDX - expectedDX;



    /**        yController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        double discY = actualDY - expectedDY;

     * UNLOCK - Exit lockdown mode

     */        headingController = new PIDController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);        double discH = normalizeAngle(actualDH - expectedDH);

    public void unlock() {

        if (inLockdown) {

            inLockdown = false;

            pinpoint.update();        // Initialize pose        if (Math.abs(discX) < 0.05) discX = 0;

            Pose2D pose = pinpoint.getPosition();

            lastX = pose.getX(DistanceUnit.INCH);        pinpoint.update();        if (Math.abs(discY) < 0.05) discY = 0;

            lastY = pose.getY(DistanceUnit.INCH);

            lastHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);        lastPose = getPoseFromPinpoint();        if (Math.abs(discH) < 0.01) discH = 0;

            lastTimeNanos = System.nanoTime();

        }        loopTimer.reset();

    }

    }        double corrX = clamp(-xController.calculate(discX), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

    /**

     * Check if in lockdown mode        double corrY = clamp(-yController.calculate(discY), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

     */

    public boolean isLocked() {    /**        double corrH = clamp(-headingController.calculate(discH), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

        return inLockdown;

    }     * DEFENSE MODE - Drive while resisting pushes



    /**     */        double finalStrafe = strafe + corrX;

     * Get current pose

     */    public void defense(double strafe, double forward, double rotate) {        double finalForward = forward + corrY;

    public Pose2d getPose() {

        pinpoint.update();        if (inLockdown) {        double finalRotate = rotate + corrH;

        Pose2D pose = pinpoint.getPosition();

        return new Pose2d(            holdPosition();

            pose.getX(DistanceUnit.INCH),

            pose.getY(DistanceUnit.INCH),            return;        double max = Math.max(1.0, Math.abs(finalStrafe) + Math.abs(finalForward) + Math.abs(finalRotate));

            normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset)

        );        }        finalStrafe /= max;

    }

        finalForward /= max;

    /**

     * Reset heading to zero (for field-centric)        pinpoint.update();        finalRotate /= max;

     */

    public void resetHeading() {        Pose2d currentPose = getPoseFromPinpoint();

        headingOffset = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

    }                if (NovadSetup.USE_FIELD_CENTRIC) {



    /**        double dt = getDeltaTime();            drivetrain.driveFieldCentric(finalStrafe, finalForward, finalRotate, currentPose.heading);

     * Update PIDF coefficients (for Dashboard tuning)

     */        updatePIDGains();        } else {

    public void setTranslationalPIDF(PIDFCoefficients coeffs) {

        this.translationalPIDF = coeffs;            drivetrain.drive(finalStrafe, finalForward, finalRotate);

    }

        // What actually happened        }

    public void setHeadingPIDF(PIDFCoefficients coeffs) {

        this.headingPIDF = coeffs;        double actualDX = currentPose.x - lastPose.x;

    }

        double actualDY = currentPose.y - lastPose.y;        lastPose = currentPose;

    // ═══════════════════════════════════════════════════════════════════════════

    //                           PRIVATE METHODS        double actualDH = normalizeAngle(currentPose.heading - lastPose.heading);    }

    // ═══════════════════════════════════════════════════════════════════════════



    private void holdPosition() {

        pinpoint.update();        // What we expected based on joystick    /**

        Pose2D pose = pinpoint.getPosition();

        double currentX = pose.getX(DistanceUnit.INCH);        double expectedVX = strafe * MAX_VELOCITY;     * LOCKDOWN - Robot becomes immovable.

        double currentY = pose.getY(DistanceUnit.INCH);

        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);        double expectedVY = forward * MAX_VELOCITY;     */



        double errorX = lockX - currentX;        double expectedVH = rotate * MAX_ANGULAR_VEL;    public void lockdown() {

        double errorY = lockY - currentY;

        double errorH = normalizeAngle(lockHeading - currentHeading);        if (!inLockdown) {



        // Update gains for live tuning        if (settings.getUseFieldCentric()) {            localizer.update();

        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);            double cos = Math.cos(currentPose.heading);            lockdownTarget = localizer.getPose();

        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

            double sin = Math.sin(currentPose.heading);            inLockdown = true;

        double corrX = clamp(xController.calculate(errorX), -maxPower, maxPower);

        double corrY = clamp(yController.calculate(errorY), -maxPower, maxPower);            double temp = expectedVX;            xController.reset();

        double corrH = clamp(headingController.calculate(errorH), -maxPower, maxPower);

            expectedVX = temp * cos - expectedVY * sin;            yController.reset();

        driveMotors(corrX, corrY, corrH, currentHeading);

    }            expectedVY = temp * sin + expectedVY * cos;            headingController.reset();



    private void driveMotors(double strafe, double forward, double rotate, double heading) {        }        }

        // Field-centric transform

        if (useFieldCentric) {        holdPosition();

            double cos = Math.cos(-heading);

            double sin = Math.sin(-heading);        double expectedDX = expectedVX * dt;    }

            double temp = strafe;

            strafe = temp * cos - forward * sin;        double expectedDY = expectedVY * dt;

            forward = temp * sin + forward * cos;

        }        double expectedDH = expectedVH * dt;    /**



        // Mecanum kinematics     * Exit lockdown.

        double fl = forward + strafe + rotate;

        double fr = forward - strafe - rotate;        // Discrepancy = external push     */

        double bl = forward - strafe + rotate;

        double br = forward + strafe - rotate;        double discX = actualDX - expectedDX;    public void unlock() {



        // Normalize        double discY = actualDY - expectedDY;        inLockdown = false;

        double max = Math.max(1.0, Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

                double discH = normalizeAngle(actualDH - expectedDH);        localizer.update();

        // Direct motor write - fastest possible

        frontLeft.setPower(fl / max);        lastPose = localizer.getPose();

        frontRight.setPower(fr / max);

        backLeft.setPower(bl / max);        // Deadzone    }

        backRight.setPower(br / max);

    }        if (Math.abs(discX) < 0.05) discX = 0;



    private static double normalizeAngle(double angle) {        if (Math.abs(discY) < 0.05) discY = 0;    public boolean isLocked() {

        while (angle > Math.PI) angle -= 2.0 * Math.PI;

        while (angle < -Math.PI) angle += 2.0 * Math.PI;        if (Math.abs(discH) < 0.01) discH = 0;        return inLockdown;

        return angle;

    }    }



    private static double clamp(double val, double min, double max) {        // Calculate corrections

        return val < min ? min : (val > max ? max : val);

    }        double maxPower = settings.getMaxCorrectionPower();    public Pose2d getPose() {

}

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
