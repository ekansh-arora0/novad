package org.firstinspires.ftc.teamcode.novad;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;
import org.firstinspires.ftc.teamcode.NovadSetup;

/**
 * NOVAD - FTC Defense Library
 * 
 * API:
 *   novad.defense(strafe, forward, rotate);  // Drive with push resistance
 *   novad.lockdown();                         // Immovable mode
 *   novad.unlock();                           // Exit lockdown
 */
public class Novad {

    public static final String VERSION = "2.0.0";

    private final Localizer localizer;
    private final MecanumDrivetrain drivetrain;

    private final PIDController xController;
    private final PIDController yController;
    private final PIDController headingController;

    private Pose2d lastPose;
    private Pose2d lockdownTarget = null;
    
    private final ElapsedTime loopTimer = new ElapsedTime();
    private double lastLoopTime = 0;

    private boolean inLockdown = false;

    private static final double MAX_VELOCITY = 40.0;
    private static final double MAX_ANGULAR_VEL = Math.PI;

    public static Novad create(HardwareMap hardwareMap) {
        Localizer localizer;

        if (NovadSetup.USE_PINPOINT) {
            localizer = new PinpointLocalizer(hardwareMap, NovadSetup.PINPOINT_NAME);
        } else if (NovadSetup.USE_THREE_WHEEL) {
            localizer = new ThreeWheelLocalizer(
                hardwareMap,
                NovadSetup.LEFT_ENCODER,
                NovadSetup.RIGHT_ENCODER,
                NovadSetup.CENTER_ENCODER,
                NovadSetup.TICKS_TO_INCHES,
                NovadSetup.TRACK_WIDTH,
                NovadSetup.CENTER_OFFSET,
                NovadSetup.LEFT_ENCODER_REVERSED,
                NovadSetup.RIGHT_ENCODER_REVERSED,
                NovadSetup.CENTER_ENCODER_REVERSED
            );
        } else {
            throw new IllegalStateException("Enable USE_PINPOINT or USE_THREE_WHEEL in NovadSetup");
        }

        return new Novad(localizer, new MecanumDrivetrain(hardwareMap));
    }

    public Novad(Localizer localizer, MecanumDrivetrain drivetrain) {
        this.localizer = localizer;
        this.drivetrain = drivetrain;

        this.xController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);
        this.yController = new PIDController(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);
        this.headingController = new PIDController(NovadSetup.HEADING_P, NovadSetup.HEADING_I, NovadSetup.HEADING_D, NovadSetup.HEADING_F);

        localizer.update();
        lastPose = localizer.getPose();
        loopTimer.reset();
    }

    /**
     * DEFENSE MODE - Call in your loop.
     * Drives robot while resisting external pushes.
     */
    public void defense(double strafe, double forward, double rotate) {
        if (inLockdown) {
            holdPosition();
            return;
        }

        localizer.update();
        Pose2d currentPose = localizer.getPose();
        
        double dt = getDeltaTime();
        updatePIDGains();

        double actualDX = currentPose.x - lastPose.x;
        double actualDY = currentPose.y - lastPose.y;
        double actualDH = normalizeAngle(currentPose.heading - lastPose.heading);

        double expectedVX = strafe * MAX_VELOCITY;
        double expectedVY = forward * MAX_VELOCITY;
        double expectedVH = rotate * MAX_ANGULAR_VEL;

        if (NovadSetup.USE_FIELD_CENTRIC) {
            double cos = Math.cos(currentPose.heading);
            double sin = Math.sin(currentPose.heading);
            double temp = expectedVX;
            expectedVX = temp * cos - expectedVY * sin;
            expectedVY = temp * sin + expectedVY * cos;
        }

        double expectedDX = expectedVX * dt;
        double expectedDY = expectedVY * dt;
        double expectedDH = expectedVH * dt;

        double discX = actualDX - expectedDX;
        double discY = actualDY - expectedDY;
        double discH = normalizeAngle(actualDH - expectedDH);

        if (Math.abs(discX) < 0.05) discX = 0;
        if (Math.abs(discY) < 0.05) discY = 0;
        if (Math.abs(discH) < 0.01) discH = 0;

        double corrX = clamp(-xController.calculate(discX), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);
        double corrY = clamp(-yController.calculate(discY), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);
        double corrH = clamp(-headingController.calculate(discH), -NovadSetup.MAX_POWER, NovadSetup.MAX_POWER);

        double finalStrafe = strafe + corrX;
        double finalForward = forward + corrY;
        double finalRotate = rotate + corrH;

        double max = Math.max(1.0, Math.abs(finalStrafe) + Math.abs(finalForward) + Math.abs(finalRotate));
        finalStrafe /= max;
        finalForward /= max;
        finalRotate /= max;

        if (NovadSetup.USE_FIELD_CENTRIC) {
            drivetrain.driveFieldCentric(finalStrafe, finalForward, finalRotate, currentPose.heading);
        } else {
            drivetrain.drive(finalStrafe, finalForward, finalRotate);
        }

        lastPose = currentPose;
    }

    /**
     * LOCKDOWN - Robot becomes immovable.
     */
    public void lockdown() {
        if (!inLockdown) {
            localizer.update();
            lockdownTarget = localizer.getPose();
            inLockdown = true;
            xController.reset();
            yController.reset();
            headingController.reset();
        }
        holdPosition();
    }

    /**
     * Exit lockdown.
     */
    public void unlock() {
        inLockdown = false;
        localizer.update();
        lastPose = localizer.getPose();
    }

    public boolean isLocked() {
        return inLockdown;
    }

    public Pose2d getPose() {
        return localizer.getPose();
    }

    public void resetHeading() {
        localizer.update();
        Pose2d p = localizer.getPose();
        lastPose = new Pose2d(p.x, p.y, 0);
        if (inLockdown && lockdownTarget != null) {
            lockdownTarget = new Pose2d(lockdownTarget.x, lockdownTarget.y, 0);
        }
    }

    private void holdPosition() {
        localizer.update();
        Pose2d current = localizer.getPose();
        updatePIDGains();

        double errX = lockdownTarget.x - current.x;
        double errY = lockdownTarget.y - current.y;
        double errH = normalizeAngle(lockdownTarget.heading - current.heading);

        double corrX = clamp(xController.calculate(errX), -1.0, 1.0);
        double corrY = clamp(yController.calculate(errY), -1.0, 1.0);
        double corrH = clamp(headingController.calculate(errH), -1.0, 1.0);

        if (NovadSetup.USE_FIELD_CENTRIC) {
            drivetrain.driveFieldCentric(corrX, corrY, corrH, current.heading);
        } else {
            drivetrain.drive(corrX, corrY, corrH);
        }
    }

    private double getDeltaTime() {
        double now = loopTimer.seconds();
        double dt = now - lastLoopTime;
        lastLoopTime = now;
        return (dt <= 0 || dt > 0.5) ? 0.02 : dt;
    }

    private void updatePIDGains() {
        xController.setGains(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);
        yController.setGains(NovadSetup.TRANS_P, NovadSetup.TRANS_I, NovadSetup.TRANS_D, NovadSetup.TRANS_F);
        headingController.setGains(NovadSetup.HEADING_P, NovadSetup.HEADING_I, NovadSetup.HEADING_D, NovadSetup.HEADING_F);
    }

    private double normalizeAngle(double a) {
        while (a > Math.PI) a -= 2 * Math.PI;
        while (a < -Math.PI) a += 2 * Math.PI;
        return a;
    }

    private double clamp(double v, double min, double max) {
        return Math.max(min, Math.min(max, v));
    }
}
