package org.firstinspires.ftc.teamcode.novad;package org.firstinspires.ftc.teamcode.novad;package org.firstinspires.ftc.teamcode.novad;package org.firstinspires.ftc.teamcode.novad;



import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.DcMotor;

import com.qualcomm.robotcore.hardware.DcMotorEx;import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;

import com.qualcomm.robotcore.hardware.Gamepad;

import com.qualcomm.robotcore.hardware.HardwareMap;import com.qualcomm.robotcore.hardware.DcMotor;



import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;import com.qualcomm.robotcore.hardware.DcMotorEx;import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;import com.qualcomm.robotcore.hardware.HardwareMap;



/**import com.qualcomm.robotcore.hardware.DcMotor;import com.qualcomm.robotcore.util.ElapsedTime;

 * NOVAD - FTC Defense Library v2.2.0

 * import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

 * Ultra-low latency defense system with <20ms reaction time.

 * Now tracks ACTUAL motor commands, not just joystick input.import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;import com.qualcomm.robotcore.hardware.DcMotorEx;import org.firstinspires.ftc.teamcode.NovadSetup;

 * This handles button-triggered movements (auto-align, macros, etc.)

 * import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;

 * KEY INSIGHT: We compare what motors are COMMANDED to do vs what

 * the robot ACTUALLY does. Any discrepancy = external push.import com.qualcomm.robotcore.hardware.HardwareMap;

 * 

 * Usage:/**

 *   Novad novad = Constants.createNovad(hardwareMap);

 *    * NOVAD - FTC Defense Libraryimport com.qualcomm.robotcore.util.ElapsedTime;/**

 *   // In your loop - pass the gamepad for full input tracking

 *   novad.defense(gamepad1);  // Auto-reads all inputs * 

 *   

 *   // OR use manual mode if you have custom motor commands * Ultra-low latency defense system with <20ms reaction time. * NOVAD - FTC Defense Library

 *   novad.defenseWithMotors(lf, rf, lb, rb);  // Pass your motor powers

 *    * Uses Pinpoint odometry for precise position tracking.

 *   novad.lockdown();  // Become immovable

 *   novad.unlock();    // Exit lockdown * import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit; * 

 */

public class Novad { * Usage:



    public static final String VERSION = "2.2.0"; *   Novad novad = Constants.createNovad(hardwareMap);import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit; * API:



    // Hardware *   novad.defense(strafe, forward, rotate);  // In loop

    private final GoBildaPinpointDriver pinpoint;

    private final DcMotorEx frontLeft, frontRight, backLeft, backRight; *   novad.lockdown();                         // Become immovableimport org.firstinspires.ftc.robotcore.external.navigation.Pose2D; *   novad.defense(strafe, forward, rotate);  // Drive with push resistance



    // Controllers *   novad.unlock();                           // Exit lockdown

    private final PIDController xController;

    private final PIDController yController; */ *   novad.lockdown();                         // Immovable mode

    private final PIDController headingController;

public class Novad {

    // Settings

    private final double maxPower;/** *   novad.unlock();                           // Exit lockdown

    private final boolean useFieldCentric;

    private PIDFCoefficients translationalPIDF;    public static final String VERSION = "2.1.0";

    private PIDFCoefficients headingPIDF;

 * NOVAD - FTC Defense Library */

    // State - preallocated

    private double lastX, lastY, lastHeading;    // Hardware - direct references for speed

    private double lockX, lockY, lockHeading;

    private boolean inLockdown = false;    private final GoBildaPinpointDriver pinpoint; * public class Novad {

    private double headingOffset = 0;

    private long lastTimeNanos;    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;



    // Motor command tracking (for button-triggered movements) * Usage:

    private double lastCommandedLF = 0, lastCommandedRF = 0;

    private double lastCommandedLB = 0, lastCommandedRB = 0;    // Controllers



    // Constants    private final PIDController xController; *   Novad novad = Constants.createNovad(hardwareMap);    public static final String VERSION = "2.0.0";

    private static final double MAX_VELOCITY = 60.0;       // inches/sec

    private static final double MAX_ANGULAR_VEL = Math.PI; // rad/sec    private final PIDController yController;

    private static final double POSITION_DEADZONE = 0.02;  // inches

    private static final double HEADING_DEADZONE = 0.005;  // radians    private final PIDController headingController; *   



    /**

     * Create Novad using NovadBuilder

     */    // Settings - cached for speed *   // In loop:    private final Localizer localizer;

    Novad(HardwareMap hardwareMap,

          MecanumConstants drive,    private final double maxPower;

          PinpointConstants localizer,

          PIDFCoefficients translationalPIDF,    private final boolean useFieldCentric; *   novad.defense(strafe, forward, rotate);  // Drive with push resistance    private final MecanumDrivetrain drivetrain;

          PIDFCoefficients headingPIDF,

          DefenseSettings settings) {    private PIDFCoefficients translationalPIDF;

        

        this.translationalPIDF = translationalPIDF;    private PIDFCoefficients headingPIDF; *   novad.lockdown();                         // Become immovable

        this.headingPIDF = headingPIDF;

        this.maxPower = settings.getMaxCorrectionPower();

        this.useFieldCentric = settings.getUseFieldCentric();

    // State - preallocated to avoid GC *   novad.unlock();                           // Exit lockdown    private final PIDController xController;

        // Initialize Pinpoint

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizer.getHardwareMapName());    private double lastX, lastY, lastHeading;

        

        if (localizer.isUsingCustomResolution()) {    private double lockX, lockY, lockHeading; */    private final PIDController yController;

            pinpoint.setEncoderResolution(localizer.getCustomEncoderResolution());

        } else {    private boolean inLockdown = false;

            pinpoint.setEncoderResolution(localizer.getEncoderResolution());

        }    private double headingOffset = 0;public class Novad {    private final PIDController headingController;

        

        pinpoint.setEncoderDirections(    private long lastTimeNanos;

            localizer.getForwardEncoderDirection(),

            localizer.getStrafeEncoderDirection()

        );

        pinpoint.resetPosAndIMU();    // Constants



        // Initialize motors    private static final double MAX_VELOCITY = 60.0;       // inches/sec (tunable)    public static final String VERSION = "2.0.0";    private Pose2d lastPose;

        frontLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftFrontMotorName());

        frontRight = hardwareMap.get(DcMotorEx.class, drive.getRightFrontMotorName());    private static final double MAX_ANGULAR_VEL = Math.PI; // rad/sec

        backLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftRearMotorName());

        backRight = hardwareMap.get(DcMotorEx.class, drive.getRightRearMotorName());    private static final double POSITION_DEADZONE = 0.02;  // inches    private Pose2d lockdownTarget = null;



        frontLeft.setDirection(drive.getLeftFrontMotorDirection());    private static final double HEADING_DEADZONE = 0.005;  // radians

        frontRight.setDirection(drive.getRightFrontMotorDirection());

        backLeft.setDirection(drive.getLeftRearMotorDirection());    // Hardware    

        backRight.setDirection(drive.getRightRearMotorDirection());

    /**

        DcMotor.ZeroPowerBehavior brake = drive.getUseBrakeMode()

            ? DcMotor.ZeroPowerBehavior.BRAKE     * Create Novad using NovadBuilder    private final GoBildaPinpointDriver pinpoint;    private final ElapsedTime loopTimer = new ElapsedTime();

            : DcMotor.ZeroPowerBehavior.FLOAT;

     */

        frontLeft.setZeroPowerBehavior(brake);

        frontRight.setZeroPowerBehavior(brake);    Novad(HardwareMap hardwareMap,    private final DcMotorEx frontLeft, frontRight, backLeft, backRight;    private double lastLoopTime = 0;

        backLeft.setZeroPowerBehavior(brake);

        backRight.setZeroPowerBehavior(brake);          MecanumConstants drive,



        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);          PinpointConstants localizer,

        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);          PIDFCoefficients translationalPIDF,

        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

          PIDFCoefficients headingPIDF,    // Controllers    private boolean inLockdown = false;

        // Initialize PID controllers

        xController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);          DefenseSettings settings) {

        yController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        headingController = new PIDController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);            private final PIDController xController;



        // Initialize state        this.translationalPIDF = translationalPIDF;

        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();        this.headingPIDF = headingPIDF;    private final PIDController yController;    private static final double MAX_VELOCITY = 40.0;

        lastX = pose.getX(DistanceUnit.INCH);

        lastY = pose.getY(DistanceUnit.INCH);        this.maxPower = settings.getMaxCorrectionPower();

        lastHeading = pose.getHeading(AngleUnit.RADIANS);

        lastTimeNanos = System.nanoTime();        this.useFieldCentric = settings.getUseFieldCentric();    private final PIDController headingController;    private static final double MAX_ANGULAR_VEL = Math.PI;

    }



    // ========================================================================

    // PRIMARY API - Use these methods        // Initialize Pinpoint - bulk read mode for speed

    // ========================================================================

        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizer.getHardwareMapName());

    /**

     * DEFENSE MODE - Gamepad Input (Recommended)            // Settings    public static Novad create(HardwareMap hardwareMap) {

     * 

     * Reads ALL gamepad inputs to understand what the driver wants.        if (localizer.isUsingCustomResolution()) {

     * Handles joystick driving AND button-triggered movements.

     *             pinpoint.setEncoderResolution(localizer.getCustomEncoderResolution());    private final DefenseSettings settings;        Localizer localizer;

     * @param gamepad The gamepad controlling the robot

     */        } else {

    public void defense(Gamepad gamepad) {

        if (inLockdown) {            pinpoint.setEncoderResolution(localizer.getEncoderResolution());    private final PIDFCoefficients translationalPIDF;

            holdPosition();

            return;        }

        }

            private final PIDFCoefficients headingPIDF;        if (NovadSetup.USE_PINPOINT) {

        // Calculate what driver wants from joystick

        double strafe = gamepad.left_stick_x;        pinpoint.setEncoderDirections(

        double forward = -gamepad.left_stick_y;

        double rotate = gamepad.right_stick_x;            localizer.getForwardEncoderDirection(),            localizer = new PinpointLocalizer(hardwareMap, NovadSetup.PINPOINT_NAME);



        // Field-centric transform            localizer.getStrafeEncoderDirection()

        double heading = getCurrentHeading();

        if (useFieldCentric) {        );    // State        } else if (NovadSetup.USE_THREE_WHEEL) {

            double cos = Math.cos(heading);

            double sin = Math.sin(heading);        pinpoint.resetPosAndIMU();

            double temp = strafe;

            strafe = temp * cos - forward * sin;    private Pose2d lastPose;            localizer = new ThreeWheelLocalizer(

            forward = temp * sin + forward * cos;

        }        // Initialize motors - direct access for minimal latency



        // Calculate mecanum motor powers from joystick        frontLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftFrontMotorName());    private Pose2d lockdownTarget = null;                hardwareMap,

        double lf = forward + strafe + rotate;

        double rf = forward - strafe - rotate;        frontRight = hardwareMap.get(DcMotorEx.class, drive.getRightFrontMotorName());

        double lb = forward - strafe + rotate;

        double rb = forward + strafe - rotate;        backLeft = hardwareMap.get(DcMotorEx.class, drive.getLeftRearMotorName());    private boolean inLockdown = false;                NovadSetup.LEFT_ENCODER,



        // Normalize        backRight = hardwareMap.get(DcMotorEx.class, drive.getRightRearMotorName());

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb))));

        if (max > 1.0) {    private double headingOffset = 0;                NovadSetup.RIGHT_ENCODER,

            lf /= max;

            rf /= max;        frontLeft.setDirection(drive.getLeftFrontMotorDirection());

            lb /= max;

            rb /= max;        frontRight.setDirection(drive.getRightFrontMotorDirection());                    NovadSetup.CENTER_ENCODER,

        }

        backLeft.setDirection(drive.getLeftRearMotorDirection());

        // Now apply defense using motor powers

        defenseWithMotors(lf, rf, lb, rb);        backRight.setDirection(drive.getRightRearMotorDirection());    private final ElapsedTime loopTimer = new ElapsedTime();                NovadSetup.TICKS_TO_INCHES,

    }



    /**

     * DEFENSE MODE - Joystick Values (Legacy)        DcMotor.ZeroPowerBehavior brake = drive.getUseBrakeMode()    private double lastLoopTime = 0;                NovadSetup.TRACK_WIDTH,

     * 

     * Use this if you compute your own strafe/forward/rotate values.            ? DcMotor.ZeroPowerBehavior.BRAKE

     * WARNING: Won't catch button-triggered movements!

     */            : DcMotor.ZeroPowerBehavior.FLOAT;                NovadSetup.CENTER_OFFSET,

    public void defense(double strafe, double forward, double rotate) {

        if (inLockdown) {

            holdPosition();

            return;        frontLeft.setZeroPowerBehavior(brake);    // Constants                NovadSetup.LEFT_ENCODER_REVERSED,

        }

        frontRight.setZeroPowerBehavior(brake);

        double heading = getCurrentHeading();

        if (useFieldCentric) {        backLeft.setZeroPowerBehavior(brake);    private static final double MAX_VELOCITY = 40.0;  // inches per second                NovadSetup.RIGHT_ENCODER_REVERSED,

            double cos = Math.cos(heading);

            double sin = Math.sin(heading);        backRight.setZeroPowerBehavior(brake);

            double temp = strafe;

            strafe = temp * cos - forward * sin;    private static final double MAX_ANGULAR_VEL = Math.PI;  // radians per second                NovadSetup.CENTER_ENCODER_REVERSED

            forward = temp * sin + forward * cos;

        }        // RUN_WITHOUT_ENCODER for lowest latency



        double lf = forward + strafe + rotate;        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);            );

        double rf = forward - strafe - rotate;

        double lb = forward - strafe + rotate;        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        double rb = forward + strafe - rotate;

        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);    /**        } else {

        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb))));

        if (max > 1.0) {        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

            lf /= max;

            rf /= max;     * Create Novad using builder (preferred method)            throw new IllegalStateException("Enable USE_PINPOINT or USE_THREE_WHEEL in NovadSetup");

            lb /= max;

            rb /= max;        // Initialize PID controllers

        }

        xController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);     */        }

        defenseWithMotors(lf, rf, lb, rb);

    }        yController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);



    /**        headingController = new PIDController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);    Novad(HardwareMap hardwareMap,

     * DEFENSE MODE - Direct Motor Powers (Best for custom controls)

     * 

     * Pass the motor powers you INTEND to apply. Novad will:

     * 1. Track what movement these powers should cause        // Initialize state          MecanumConstants driveConstants,        return new Novad(localizer, new MecanumDrivetrain(hardwareMap));

     * 2. Detect any discrepancy from external forces

     * 3. Apply corrections to resist pushes        pinpoint.update();

     * 4. Actually send the corrected powers to motors

     *         Pose2D pose = pinpoint.getPosition();          PinpointConstants localizerConstants,    }

     * This handles ALL edge cases including:

     * - Joystick driving        lastX = pose.getX(DistanceUnit.INCH);

     * - Button-triggered movements (auto-align, macros)

     * - Any custom motor commands        lastY = pose.getY(DistanceUnit.INCH);          PIDFCoefficients translationalPIDF,

     * 

     * @param intendedLF Intended front-left power (-1 to 1)        lastHeading = pose.getHeading(AngleUnit.RADIANS);

     * @param intendedRF Intended front-right power (-1 to 1)

     * @param intendedLB Intended back-left power (-1 to 1)        lastTimeNanos = System.nanoTime();          PIDFCoefficients headingPIDF,    public Novad(Localizer localizer, MecanumDrivetrain drivetrain) {

     * @param intendedRB Intended back-right power (-1 to 1)

     */    }

    public void defenseWithMotors(double intendedLF, double intendedRF, 

                                   double intendedLB, double intendedRB) {          DefenseSettings settings) {        this.localizer = localizer;

        if (inLockdown) {

            holdPosition();    /**

            return;

        }     * DEFENSE MODE - Drive while resisting pushes                this.drivetrain = drivetrain;



        // Get current pose     * Call this every loop iteration (~20ms or faster)

        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();     */        this.translationalPIDF = translationalPIDF;

        double currentX = pose.getX(DistanceUnit.INCH);

        double currentY = pose.getY(DistanceUnit.INCH);    public void defense(double strafe, double forward, double rotate) {

        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);

        if (inLockdown) {        this.headingPIDF = headingPIDF;        this.xController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

        // Calculate delta time

        long now = System.nanoTime();            holdPosition();

        double dt = (now - lastTimeNanos) * 1e-9;

        lastTimeNanos = now;            return;        this.settings = settings;        this.yController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);

        if (dt < 0.001) dt = 0.001;

        }

        // ACTUAL movement this frame

        double actualDX = currentX - lastX;        this.headingController = new PIDController(NovadSetup.HEADING_P, NovadSetup.HEADING_I, NovadSetup.HEADING_D, NovadSetup.HEADING_F);

        double actualDY = currentY - lastY;

        double actualDH = normalizeAngle(currentHeading - lastHeading);        // Get current pose - single I2C read



        // EXPECTED movement based on what motors were commanded        pinpoint.update();        // Initialize Pinpoint

        // Inverse kinematics: motor powers -> robot velocity

        double expStrafe = (lastCommandedLF - lastCommandedRF - lastCommandedLB + lastCommandedRB) / 4.0;        Pose2D pose = pinpoint.getPosition();

        double expForward = (lastCommandedLF + lastCommandedRF + lastCommandedLB + lastCommandedRB) / 4.0;

        double expRotate = (-lastCommandedLF + lastCommandedRF - lastCommandedLB + lastCommandedRB) / 4.0;        double currentX = pose.getX(DistanceUnit.INCH);        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, localizerConstants.getHardwareMapName());        localizer.update();



        double expDX = expStrafe * MAX_VELOCITY * dt;        double currentY = pose.getY(DistanceUnit.INCH);

        double expDY = expForward * MAX_VELOCITY * dt;

        double expDH = expRotate * MAX_ANGULAR_VEL * dt;        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);                lastPose = localizer.getPose();



        // Discrepancy = external force pushed us

        double discX = actualDX - expDX;

        double discY = actualDY - expDY;        // Calculate delta time        if (localizerConstants.isUsingCustomResolution()) {        loopTimer.reset();

        double discH = normalizeAngle(actualDH - expDH);

        long now = System.nanoTime();

        // Deadzone to prevent jitter

        if (Math.abs(discX) < POSITION_DEADZONE) discX = 0;        double dt = (now - lastTimeNanos) * 1e-9;            pinpoint.setEncoderResolution(localizerConstants.getCustomEncoderResolution());    }

        if (Math.abs(discY) < POSITION_DEADZONE) discY = 0;

        if (Math.abs(discH) < HEADING_DEADZONE) discH = 0;        lastTimeNanos = now;



        // Update PID gains (for live Dashboard tuning)        if (dt < 0.001) dt = 0.001;        } else {

        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

        // Actual movement this frame            pinpoint.setEncoderResolution(localizerConstants.getEncoderResolution());    /**

        // Calculate corrections

        double corrX = clamp(-xController.calculate(discX), -maxPower, maxPower);        double actualDX = currentX - lastX;

        double corrY = clamp(-yController.calculate(discY), -maxPower, maxPower);

        double corrH = clamp(-headingController.calculate(discH), -maxPower, maxPower);        double actualDY = currentY - lastY;        }     * DEFENSE MODE - Call in your loop.



        // Convert corrections back to motor powers        double actualDH = normalizeAngle(currentHeading - lastHeading);

        // Forward kinematics: robot velocity -> motor powers

        double corrLF = corrY + corrX + corrH;             * Drives robot while resisting external pushes.

        double corrRF = corrY - corrX - corrH;

        double corrLB = corrY - corrX + corrH;        // Expected movement based on joystick

        double corrRB = corrY + corrX - corrH;

        double expVX = strafe * MAX_VELOCITY;        pinpoint.setEncoderDirections(     */

        // Combine intended + correction

        double finalLF = intendedLF + corrLF;        double expVY = forward * MAX_VELOCITY;

        double finalRF = intendedRF + corrRF;

        double finalLB = intendedLB + corrLB;        double expVH = rotate * MAX_ANGULAR_VEL;            localizerConstants.getForwardEncoderDirection(),    public void defense(double strafe, double forward, double rotate) {

        double finalRB = intendedRB + corrRB;



        // Clamp to valid range

        finalLF = clamp(finalLF, -1.0, 1.0);        // Field-centric transform if enabled            localizerConstants.getStrafeEncoderDirection()        if (inLockdown) {

        finalRF = clamp(finalRF, -1.0, 1.0);

        finalLB = clamp(finalLB, -1.0, 1.0);        if (useFieldCentric) {

        finalRB = clamp(finalRB, -1.0, 1.0);

            double cos = Math.cos(currentHeading);        );            holdPosition();

        // Apply to motors

        frontLeft.setPower(finalLF);            double sin = Math.sin(currentHeading);

        frontRight.setPower(finalRF);

        backLeft.setPower(finalLB);            double temp = expVX;                    return;

        backRight.setPower(finalRB);

            expVX = temp * cos - expVY * sin;

        // Store commanded values for next iteration

        lastCommandedLF = finalLF;            expVY = temp * sin + expVY * cos;        pinpoint.resetPosAndIMU();        }

        lastCommandedRF = finalRF;

        lastCommandedLB = finalLB;        }

        lastCommandedRB = finalRB;



        // Update state

        lastX = currentX;        double expDX = expVX * dt;

        lastY = currentY;

        lastHeading = currentHeading;        double expDY = expVY * dt;        // Initialize motors        localizer.update();

    }

        double expDH = expVH * dt;

    /**

     * LOCKDOWN MODE - Become completely immovable        frontLeft = hardwareMap.get(DcMotorEx.class, driveConstants.getLeftFrontMotorName());        Pose2d currentPose = localizer.getPose();

     */

    public void lockdown() {        // Discrepancy = external force

        if (!inLockdown) {

            pinpoint.update();        double discX = actualDX - expDX;        frontRight = hardwareMap.get(DcMotorEx.class, driveConstants.getRightFrontMotorName());        

            Pose2D pose = pinpoint.getPosition();

            lockX = pose.getX(DistanceUnit.INCH);        double discY = actualDY - expDY;

            lockY = pose.getY(DistanceUnit.INCH);

            lockHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);        double discH = normalizeAngle(actualDH - expDH);        backLeft = hardwareMap.get(DcMotorEx.class, driveConstants.getLeftRearMotorName());        double dt = getDeltaTime();

            

            xController.reset();

            yController.reset();

            headingController.reset();        // Deadzone to prevent jitter        backRight = hardwareMap.get(DcMotorEx.class, driveConstants.getRightRearMotorName());        updatePIDGains();

            

            inLockdown = true;        if (Math.abs(discX) < POSITION_DEADZONE) discX = 0;

        }

        holdPosition();        if (Math.abs(discY) < POSITION_DEADZONE) discY = 0;

    }

        if (Math.abs(discH) < HEADING_DEADZONE) discH = 0;

    /**

     * Exit lockdown mode        frontLeft.setDirection(driveConstants.getLeftFrontMotorDirection());        double actualDX = currentPose.x - lastPose.x;

     */

    public void unlock() {        // Update PID gains (for live Dashboard tuning)

        inLockdown = false;

        lastCommandedLF = 0;        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        frontRight.setDirection(driveConstants.getRightFrontMotorDirection());        double actualDY = currentPose.y - lastPose.y;

        lastCommandedRF = 0;

        lastCommandedLB = 0;        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        lastCommandedRB = 0;

    }        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);        backLeft.setDirection(driveConstants.getLeftRearMotorDirection());        double actualDH = normalizeAngle(currentPose.heading - lastPose.heading);



    /**

     * Check if in lockdown

     */        // Calculate corrections        backRight.setDirection(driveConstants.getRightRearMotorDirection());

    public boolean isLocked() {

        return inLockdown;        double corrX = clamp(-xController.calculate(discX), -maxPower, maxPower);

    }

        double corrY = clamp(-yController.calculate(discY), -maxPower, maxPower);        double expectedVX = strafe * MAX_VELOCITY;

    /**

     * Get current robot pose        double corrH = clamp(-headingController.calculate(discH), -maxPower, maxPower);

     */

    public Pose2d getPose() {        DcMotor.ZeroPowerBehavior behavior = driveConstants.getUseBrakeMode()        double expectedVY = forward * MAX_VELOCITY;

        double heading = getCurrentHeading();

        return new Pose2d(lastX, lastY, heading);        // Combine driver input + correction

    }

        double finalStrafe = strafe + corrX;            ? DcMotor.ZeroPowerBehavior.BRAKE        double expectedVH = rotate * MAX_ANGULAR_VEL;

    /**

     * Reset heading to zero (field-centric calibration)        double finalForward = forward + corrY;

     */

    public void resetHeading() {        double finalRotate = rotate + corrH;            : DcMotor.ZeroPowerBehavior.FLOAT;

        pinpoint.update();

        headingOffset = pinpoint.getPosition().getHeading(AngleUnit.RADIANS);

    }

        // Normalize and drive        if (NovadSetup.USE_FIELD_CENTRIC) {

    /**

     * Reset position to (0, 0, 0)        driveMotors(finalStrafe, finalForward, finalRotate, currentHeading);

     */

    public void resetPosition() {        frontLeft.setZeroPowerBehavior(behavior);            double cos = Math.cos(currentPose.heading);

        pinpoint.resetPosAndIMU();

        headingOffset = 0;        // Update state

        lastX = 0;

        lastY = 0;        lastX = currentX;        frontRight.setZeroPowerBehavior(behavior);            double sin = Math.sin(currentPose.heading);

        lastHeading = 0;

    }        lastY = currentY;



    /**        lastHeading = currentHeading;        backLeft.setZeroPowerBehavior(behavior);            double temp = expectedVX;

     * Update PIDF coefficients (for live tuning)

     */    }

    public void setTranslationalPIDF(PIDFCoefficients pidf) {

        this.translationalPIDF = pidf;        backRight.setZeroPowerBehavior(behavior);            expectedVX = temp * cos - expectedVY * sin;

    }

    /**

    public void setHeadingPIDF(PIDFCoefficients pidf) {

        this.headingPIDF = pidf;     * LOCKDOWN - Become completely immovable            expectedVY = temp * sin + expectedVY * cos;

    }

     */

    /**

     * Get loop timing for debugging    public void lockdown() {        frontLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        }

     */

    public double getLoopTimeMs() {        if (!inLockdown) {

        return (System.nanoTime() - lastTimeNanos) / 1_000_000.0;

    }            pinpoint.update();        frontRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);



    // ========================================================================            Pose2D pose = pinpoint.getPosition();

    // INTERNAL METHODS

    // ========================================================================            lockX = pose.getX(DistanceUnit.INCH);        backLeft.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        double expectedDX = expectedVX * dt;



    private void holdPosition() {            lockY = pose.getY(DistanceUnit.INCH);

        pinpoint.update();

        Pose2D pose = pinpoint.getPosition();            lockHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);        backRight.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);        double expectedDY = expectedVY * dt;

        double currentX = pose.getX(DistanceUnit.INCH);

        double currentY = pose.getY(DistanceUnit.INCH);            inLockdown = true;

        double currentHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);

            xController.reset();        double expectedDH = expectedVH * dt;

        double errorX = lockX - currentX;

        double errorY = lockY - currentY;            yController.reset();

        double errorH = normalizeAngle(lockHeading - currentHeading);

            headingController.reset();        // Initialize PID controllers

        xController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);

        yController.setPIDF(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        }

        headingController.setPIDF(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);

    }        xController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        double discX = actualDX - expectedDX;

        double corrX = clamp(xController.calculate(-errorX), -1.0, 1.0);

        double corrY = clamp(yController.calculate(-errorY), -1.0, 1.0);

        double corrH = clamp(headingController.calculate(-errorH), -1.0, 1.0);

    /**        yController = new PIDController(translationalPIDF.p, translationalPIDF.i, translationalPIDF.d, translationalPIDF.f);        double discY = actualDY - expectedDY;

        double lf = corrY + corrX + corrH;

        double rf = corrY - corrX - corrH;     * UNLOCK - Exit lockdown mode

        double lb = corrY - corrX + corrH;

        double rb = corrY + corrX - corrH;     */        headingController = new PIDController(headingPIDF.p, headingPIDF.i, headingPIDF.d, headingPIDF.f);        double discH = normalizeAngle(actualDH - expectedDH);



        double max = Math.max(Math.abs(lf), Math.max(Math.abs(rf), Math.max(Math.abs(lb), Math.abs(rb))));    public void unlock() {

        if (max > 1.0) {

            lf /= max;        if (inLockdown) {

            rf /= max;

            lb /= max;            inLockdown = false;

            rb /= max;

        }            pinpoint.update();        // Initialize pose        if (Math.abs(discX) < 0.05) discX = 0;



        frontLeft.setPower(lf);            Pose2D pose = pinpoint.getPosition();

        frontRight.setPower(rf);

        backLeft.setPower(lb);            lastX = pose.getX(DistanceUnit.INCH);        pinpoint.update();        if (Math.abs(discY) < 0.05) discY = 0;

        backRight.setPower(rb);

            lastY = pose.getY(DistanceUnit.INCH);

        lastX = currentX;

        lastY = currentY;            lastHeading = normalizeAngle(pose.getHeading(AngleUnit.RADIANS) - headingOffset);        lastPose = getPoseFromPinpoint();        if (Math.abs(discH) < 0.01) discH = 0;

        lastHeading = currentHeading;

    }            lastTimeNanos = System.nanoTime();



    private double getCurrentHeading() {        }        loopTimer.reset();

        return normalizeAngle(lastHeading);

    }    }



    private static double normalizeAngle(double angle) {    }        double corrX = clamp(-xController.calculate(discX), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

        while (angle > Math.PI) angle -= 2 * Math.PI;

        while (angle < -Math.PI) angle += 2 * Math.PI;    /**

        return angle;

    }     * Check if in lockdown mode        double corrY = clamp(-yController.calculate(discY), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);



    private static double clamp(double value, double min, double max) {     */

        return Math.max(min, Math.min(max, value));

    }    public boolean isLocked() {    /**        double corrH = clamp(-headingController.calculate(discH), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

}

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
