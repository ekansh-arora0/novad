package org.firstinspires.ftc.teamcode;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.telemetry.MultipleTelemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

import org.firstinspires.ftc.teamcode.novad.Novad;
import org.firstinspires.ftc.teamcode.novad.adapters.MecanumDriveAdapter;
import org.firstinspires.ftc.teamcode.novad.adapters.PinpointOdometry;
import org.firstinspires.ftc.teamcode.novad.adapters.ThreeWheelOdometry;
import org.firstinspires.ftc.teamcode.novad.config.NovadConfig;
import org.firstinspires.ftc.teamcode.novad.config.MotorDirection;
import org.firstinspires.ftc.teamcode.novad.config.PIDFCoefficients;
import org.firstinspires.ftc.teamcode.novad.interfaces.NovadOdometry;

/**
 * ═══════════════════════════════════════════════════════════════════════════════
 * NOVAD TELEOP EXAMPLE
 * ═══════════════════════════════════════════════════════════════════════════════
 * 
 * Complete working TeleOp with Novad defense. Uses values from NovadConstants.
 * 
 * All PIDF values are tunable live via FTC Dashboard!
 * Connect to: http://192.168.43.1:8080/dash OR https://panels.bylazar.com
 * 
 * CONTROLS:
 * - Left Stick: Drive (forward/strafe)
 * - Right Stick: Rotate
 * - A Button: Toggle position lock
 * - B Button: Toggle defense on/off
 * - X Button: Recalibrate IMU (if using Pinpoint)
 */
@TeleOp(name = "Novad TeleOp", group = "Novad")
public class NovadTeleOp extends LinearOpMode {

    @Override
    public void runOpMode() throws InterruptedException {
        // Setup FTC Dashboard telemetry
        telemetry = new MultipleTelemetry(telemetry, FtcDashboard.getInstance().getTelemetry());

        // ═══════════════════════════════════════════════════════════════════════
        // HARDWARE SETUP (using NovadConstants)
        // ═══════════════════════════════════════════════════════════════════════
        
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, NovadConstants.LEFT_FRONT_MOTOR);
        DcMotor frontRight = hardwareMap.get(DcMotor.class, NovadConstants.RIGHT_FRONT_MOTOR);
        DcMotor backLeft = hardwareMap.get(DcMotor.class, NovadConstants.LEFT_REAR_MOTOR);
        DcMotor backRight = hardwareMap.get(DcMotor.class, NovadConstants.RIGHT_REAR_MOTOR);

        // Set motor directions from constants
        frontLeft.setDirection(NovadConstants.LEFT_FRONT_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backLeft.setDirection(NovadConstants.LEFT_REAR_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        frontRight.setDirection(NovadConstants.RIGHT_FRONT_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);
        backRight.setDirection(NovadConstants.RIGHT_REAR_REVERSED ? 
            DcMotorSimple.Direction.REVERSE : DcMotorSimple.Direction.FORWARD);

        // ═══════════════════════════════════════════════════════════════════════
        // ODOMETRY SETUP (auto-selects based on NovadConstants.ODOMETRY_TYPE)
        // ═══════════════════════════════════════════════════════════════════════
        
        NovadOdometry odometry;
        
        switch (NovadConstants.ODOMETRY_TYPE) {
            case PINPOINT:
                odometry = new PinpointOdometry(
                    hardwareMap,
                    NovadConstants.PINPOINT_DEVICE_NAME,
                    NovadConstants.PINPOINT_X_OFFSET_MM,
                    NovadConstants.PINPOINT_Y_OFFSET_MM,
                    NovadConstants.PINPOINT_WHEEL_DIAMETER_MM,
                    NovadConstants.PINPOINT_ENCODER_RESOLUTION
                );
                telemetry.addLine("Using: Pinpoint Odometry");
                break;
                
            case THREE_WHEEL:
            default:
                odometry = new ThreeWheelOdometry(
                    hardwareMap,
                    NovadConstants.LEFT_ENCODER_MOTOR_PORT,
                    NovadConstants.RIGHT_ENCODER_MOTOR_PORT,
                    NovadConstants.CENTER_ENCODER_MOTOR_PORT,
                    NovadConstants.LEFT_ENCODER_REVERSED,
                    NovadConstants.RIGHT_ENCODER_REVERSED,
                    NovadConstants.CENTER_ENCODER_REVERSED,
                    NovadConstants.DEAD_WHEEL_DIAMETER_INCHES,
                    NovadConstants.DEAD_WHEEL_TICKS_PER_REV,
                    NovadConstants.TRACK_WIDTH_INCHES,
                    NovadConstants.FORWARD_OFFSET_INCHES
                );
                telemetry.addLine("Using: Three-Wheel Odometry");
                break;
        }

        // ═══════════════════════════════════════════════════════════════════════
        // DRIVETRAIN ADAPTER
        // ═══════════════════════════════════════════════════════════════════════
        
        MecanumDriveAdapter drivetrain = new MecanumDriveAdapter(
            frontLeft, frontRight, backLeft, backRight
        );

        // ═══════════════════════════════════════════════════════════════════════
        // NOVAD CONFIG (built from NovadConstants)
        // ═══════════════════════════════════════════════════════════════════════
        
        NovadConfig config = new NovadConfig.Builder()
            .leftFrontMotorName(NovadConstants.LEFT_FRONT_MOTOR)
            .leftRearMotorName(NovadConstants.LEFT_REAR_MOTOR)
            .rightFrontMotorName(NovadConstants.RIGHT_FRONT_MOTOR)
            .rightRearMotorName(NovadConstants.RIGHT_REAR_MOTOR)
            .leftFrontMotorDirection(NovadConstants.LEFT_FRONT_REVERSED ? MotorDirection.REVERSE : MotorDirection.FORWARD)
            .leftRearMotorDirection(NovadConstants.LEFT_REAR_REVERSED ? MotorDirection.REVERSE : MotorDirection.FORWARD)
            .rightFrontMotorDirection(NovadConstants.RIGHT_FRONT_REVERSED ? MotorDirection.REVERSE : MotorDirection.FORWARD)
            .rightRearMotorDirection(NovadConstants.RIGHT_REAR_REVERSED ? MotorDirection.REVERSE : MotorDirection.FORWARD)
            .translationalPIDFCoefficients(new PIDFCoefficients(
                NovadConstants.TRANS_P, NovadConstants.TRANS_I, 
                NovadConstants.TRANS_D, NovadConstants.TRANS_F))
            .headingPIDFCoefficients(new PIDFCoefficients(
                NovadConstants.HEADING_P, NovadConstants.HEADING_I, 
                NovadConstants.HEADING_D, NovadConstants.HEADING_F))
            .velocityPIDFCoefficients(new PIDFCoefficients(
                NovadConstants.VEL_P, NovadConstants.VEL_I, 
                NovadConstants.VEL_D, NovadConstants.VEL_F))
            .movementThreshold(NovadConstants.MOVEMENT_THRESHOLD)
            .headingThreshold(Math.toRadians(NovadConstants.HEADING_THRESHOLD_DEGREES))
            .maxCorrectionPower(NovadConstants.MAX_CORRECTION_POWER)
            .driverOverrideThreshold(NovadConstants.DRIVER_DEADZONE)
            .rampUpTime(NovadConstants.RAMP_UP_TIME)
            .maxIntegral(NovadConstants.MAX_INTEGRAL)
            .build();

        // ═══════════════════════════════════════════════════════════════════════
        // CREATE NOVAD
        // ═══════════════════════════════════════════════════════════════════════
        
        Novad novad = new Novad(odometry, drivetrain, config);
        
        // Configure predictive defense
        novad.setPredictiveEnabled(NovadConstants.PREDICTIVE_ENABLED);
        novad.setPredictionLookahead(NovadConstants.PREDICTION_LOOKAHEAD_MS);
        novad.setAccelTriggerThreshold(NovadConstants.ACCEL_TRIGGER_THRESHOLD);
        novad.setInstantBoostMultiplier(NovadConstants.INSTANT_BOOST_MULTIPLIER);

        // ═══════════════════════════════════════════════════════════════════════
        // INIT DISPLAY
        // ═══════════════════════════════════════════════════════════════════════
        
        telemetry.addLine("");
        telemetry.addLine("╔═══════════════════════════════════════╗");
        telemetry.addLine("║         NOVAD DEFENSE READY           ║");
        telemetry.addLine("╠═══════════════════════════════════════╣");
        telemetry.addLine("║ A = Toggle Position Lock              ║");
        telemetry.addLine("║ B = Toggle Defense On/Off             ║");
        telemetry.addLine("║ X = Recalibrate IMU                   ║");
        telemetry.addLine("╠═══════════════════════════════════════╣");
        telemetry.addLine("║ Tune values in FTC Dashboard!         ║");
        telemetry.addLine("╚═══════════════════════════════════════╝");
        telemetry.update();

        waitForStart();

        // Track button states for edge detection
        boolean lastA = false, lastB = false, lastX = false;
        boolean defenseEnabled = true;

        while (opModeIsActive()) {
            // ═══════════════════════════════════════════════════════════════════
            // UPDATE PID FROM DASHBOARD (live tuning!)
            // ═══════════════════════════════════════════════════════════════════
            
            novad.setPositionPID(NovadConstants.TRANS_P, NovadConstants.TRANS_I, NovadConstants.TRANS_D);
            novad.setHeadingPID(NovadConstants.HEADING_P, NovadConstants.HEADING_I, NovadConstants.HEADING_D);
            novad.setVelocityPID(NovadConstants.VEL_P, NovadConstants.VEL_I, NovadConstants.VEL_D);
            
            // Update predictive defense settings
            novad.setPredictiveEnabled(NovadConstants.PREDICTIVE_ENABLED);
            novad.setPredictionLookahead(NovadConstants.PREDICTION_LOOKAHEAD_MS);
            novad.setAccelTriggerThreshold(NovadConstants.ACCEL_TRIGGER_THRESHOLD);
            novad.setInstantBoostMultiplier(NovadConstants.INSTANT_BOOST_MULTIPLIER);

            // ═══════════════════════════════════════════════════════════════════
            // BUTTON HANDLING
            // ═══════════════════════════════════════════════════════════════════
            
            // A button - toggle position lock
            if (gamepad1.a && !lastA) {
                novad.togglePositionLock();
            }
            lastA = gamepad1.a;
            
            // B button - toggle defense
            if (gamepad1.b && !lastB) {
                defenseEnabled = !defenseEnabled;
                if (defenseEnabled) {
                    novad.enable();
                } else {
                    novad.disable();
                }
            }
            lastB = gamepad1.b;
            
            // X button - recalibrate IMU (Pinpoint only)
            if (gamepad1.x && !lastX) {
                if (odometry instanceof PinpointOdometry) {
                    ((PinpointOdometry) odometry).recalibrateIMU();
                    telemetry.addLine(">>> IMU Recalibrating...");
                }
            }
            lastX = gamepad1.x;

            // ═══════════════════════════════════════════════════════════════════
            // NOVAD DEFENSE - One line does it all!
            // ═══════════════════════════════════════════════════════════════════
            
            if (defenseEnabled) {
                novad.defense(
                    gamepad1.left_stick_x,     // Strafe
                    -gamepad1.left_stick_y,    // Forward (inverted)
                    gamepad1.right_stick_x     // Rotate
                );
            } else {
                // Manual drive when defense disabled
                drivetrain.drive(
                    -gamepad1.left_stick_y,
                    gamepad1.left_stick_x,
                    gamepad1.right_stick_x
                );
            }

            // ═══════════════════════════════════════════════════════════════════
            // TELEMETRY
            // ═══════════════════════════════════════════════════════════════════
            
            telemetry.addLine("═══ NOVAD STATUS ═══");
            telemetry.addData("Defense", defenseEnabled ? "🟢 ENABLED" : "🔴 DISABLED");
            telemetry.addData("Position Lock", novad.isPositionLocked() ? "🔒 LOCKED" : "🔓 UNLOCKED");
            
            telemetry.addLine("");
            telemetry.addLine("═══ PREDICTIVE DEFENSE ═══");
            telemetry.addData("Predictive", NovadConstants.PREDICTIVE_ENABLED ? "🟢 ON" : "⚪ OFF");
            telemetry.addData("Boost Active", novad.isPredictiveActive() ? "⚡ YES" : "—");
            telemetry.addData("Boost Multiplier", "%.2f", novad.getCurrentBoostMultiplier());
            telemetry.addData("Acceleration", "%.1f in/s²", novad.getAccelerationMagnitude());
            
            telemetry.addLine("");
            telemetry.addLine("═══ POSITION ═══");
            telemetry.addData("X", "%.2f in", odometry.getPosition().x);
            telemetry.addData("Y", "%.2f in", odometry.getPosition().y);
            telemetry.addData("Heading", "%.1f°", Math.toDegrees(odometry.getHeading()));
            
            telemetry.addLine("");
            telemetry.addLine("═══ PID VALUES (tune in Dashboard!) ═══");
            telemetry.addData("Trans P/I/D", "%.3f / %.3f / %.3f", 
                NovadConstants.TRANS_P, NovadConstants.TRANS_I, NovadConstants.TRANS_D);
            telemetry.addData("Heading P/I/D", "%.3f / %.3f / %.3f", 
                NovadConstants.HEADING_P, NovadConstants.HEADING_I, NovadConstants.HEADING_D);
            
            telemetry.update();
        }
    }
}
